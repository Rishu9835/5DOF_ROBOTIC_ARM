"""
coordinator.py — User-triggered pick-and-place controller.

State machine (one-shot, NOT a loop):
  IDLE → CALIBRATE → IDLE (calibrated)
  IDLE → SCAN → APPROACH → PICK → PLACE → HOME → IDLE

Workflow:
  1. User places ArUco markers 0–3 at workspace corners, clicks CALIBRATE.
     The system detects all 4 markers, computes a homography (pixel → mm).
  2. User places an object with marker 5 or 6 on the workspace, clicks SCAN.
     The system detects the object marker, maps its position to mm via the
     homography, then executes APPROACH → PICK → PLACE → HOME → IDLE.

Communicates with ESP32 via the existing hub (forward_to_esp32).
Listens for vision detections via an asyncio Queue fed by VisionEngine.

IMPORTANT: This module never imports from server.py (circular import).
It receives hub, mapper, and (optionally) vision as constructor arguments.
"""

import asyncio
import json
import logging
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional

from ik_bridge import ik_solve, make_ik_command

log = logging.getLogger("coordinator")

# ── Physical config (mm / degrees / seconds) ─────────────────
PICK_Z_APPROACH = 100.0     # mm — hover height above table
PICK_Z_GRASP    = 50.0     # mm — contact height

# Pitch convention: 0° = horizontal, positive = tilt down, negative = tilt up.
# With elbow capped at 90°, the arm MUST approach with upward wrist pitch
# to reach low positions — the long wrist chain (144.3mm) demands it.
PICK_PITCH_APPROACH = -60.0   # degrees — wrist tilted up during approach
PICK_PITCH_GRASP    = -80.0   # degrees — nearly vertical for low-Z grasp
PICK_PITCH_LIFT     = -60.0   # degrees — tilt up for clean lift

PLACE_X         = 250.0    # mm — drop-off position (robot frame)
PLACE_Y         =   0.0
PLACE_Z         =  80.0
PLACE_PITCH     = -70.0    # degrees — upward wrist for place position

GRIPPER_OPEN    = 170.0    # servo degrees — open
GRIPPER_CLOSED  =  20.0    # servo degrees — grasping

MOVE_SETTLE_S   = 1.2      # seconds to wait after each ik_move
PICK_PAUSE_S    = 0.4      # seconds to pause after gripper close before lifting
SCAN_TIMEOUT_S  = 15.0     # seconds to wait for object detection

# Sanity bounds for mm coordinates from mapper (must be within reachable workspace)
MM_X_MIN, MM_X_MAX = 200.0, 360.0
MM_Y_MIN, MM_Y_MAX = -200.0, 200.0


class State(Enum):
    IDLE      = auto()
    CALIBRATE = auto()
    SCAN      = auto()
    APPROACH  = auto()
    PICK      = auto()
    PLACE     = auto()
    HOME      = auto()


@dataclass
class PickTarget:
    label: str
    x_mm: float
    y_mm: float


class PickPlaceCoordinator:
    """User-triggered state machine for pick-and-place."""

    def __init__(self, hub, mapper=None, vision=None):
        """
        hub    : the ConnectionHub singleton from server.py
        mapper : CoordMapper instance for homography calibration
        vision : VisionEngine — used for workspace marker detection and logging
        """
        self.hub = hub
        self._mapper = mapper
        self._vision = vision
        self.state = State.IDLE
        self.target: Optional[PickTarget] = None
        self._det_queue: asyncio.Queue = asyncio.Queue(maxsize=5)
        self._calibrated = False

    def get_detection_queue(self) -> asyncio.Queue:
        """Returns the queue that VisionEngine should push detections to."""
        return self._det_queue

    # ── User-triggered actions ────────────────────────────────

    def start_calibration(self):
        """User clicked CALIBRATE — enter calibration state."""
        if self.state != State.IDLE:
            log.warning(f"Cannot calibrate: arm is busy (state={self.state.name})")
            return False
        self.state = State.CALIBRATE
        log.info("Calibration requested — looking for workspace markers 0–3")
        return True

    def start_scan(self):
        """User clicked SCAN — enter scan state (requires calibration)."""
        if self.state != State.IDLE:
            log.warning(f"Cannot scan: arm is busy (state={self.state.name})")
            return False
        if not self._calibrated and (not self._mapper or not self._mapper.is_calibrated):
            log.warning("Cannot scan: workspace not calibrated! Click CALIBRATE first.")
            return False
        self.state = State.SCAN
        log.info("Scan requested — looking for object markers (5, 6)")
        return True

    # ── Main async loop ───────────────────────────────────────

    async def run(self):
        """Main async loop — call as asyncio task."""
        log.info("Coordinator running.")
        while True:
            if self.state == State.CALIBRATE:
                await self._state_calibrate()
            elif self.state == State.SCAN:
                await self._state_scan()
            elif self.state == State.APPROACH:
                await self._state_approach()
            elif self.state == State.PICK:
                await self._state_pick()
            elif self.state == State.PLACE:
                await self._state_place()
            elif self.state == State.HOME:
                await self._state_home()
            else:
                # IDLE — just sleep
                await asyncio.sleep(0.1)

    # ── State implementations ────────────────────────────────

    async def _state_calibrate(self):
        """Detect workspace markers 0–3 and compute homography."""
        if self._vision is None or self._mapper is None:
            log.error("[CALIBRATE] No vision or mapper available")
            self.state = State.IDLE
            return

        log.info("[CALIBRATE] Waiting for all 4 workspace markers (0, 1, 2, 3)…")

        # Poll workspace pixel positions from the vision engine
        deadline = asyncio.get_event_loop().time() + 10.0  # 10s timeout
        while asyncio.get_event_loop().time() < deadline:
            ws_px = self._vision.get_workspace_pixel_points()
            if {0, 1, 2, 3}.issubset(ws_px.keys()):
                # All 4 workspace markers visible — calibrate!
                success = self._mapper.calibrate_from_markers(ws_px)
                if success:
                    self._calibrated = True
                    log.info("[CALIBRATE] ✓ Workspace calibrated successfully!")
                    self.state = State.IDLE
                    return
                else:
                    log.warning("[CALIBRATE] Homography computation failed")
                    self.state = State.IDLE
                    return

            # Drain the detection queue to keep it fresh
            while not self._det_queue.empty():
                try:
                    self._det_queue.get_nowait()
                except asyncio.QueueEmpty:
                    break

            await asyncio.sleep(0.3)

        log.warning("[CALIBRATE] Timeout — could not see all 4 workspace markers. "
                    f"Visible: {list(self._vision.get_workspace_pixel_points().keys())}")
        self.state = State.IDLE

    async def _state_scan(self):
        """Wait for vision to detect an object marker (5 or 6) with mm coords."""
        log.info("[SCAN] Waiting for object marker (5 or 6)…")

        # Drain stale queue entries first
        while not self._det_queue.empty():
            try:
                self._det_queue.get_nowait()
            except asyncio.QueueEmpty:
                break

        # Wait for a valid object detection
        deadline = asyncio.get_event_loop().time() + SCAN_TIMEOUT_S
        while asyncio.get_event_loop().time() < deadline:
            try:
                detections = await asyncio.wait_for(
                    self._det_queue.get(), timeout=2.0
                )
            except asyncio.TimeoutError:
                continue

        # Filter for object markers with valid mm coords
            objects = [d for d in detections
                       if d.role == "object" and d.center_mm is not None]
            if objects:
                best = objects[0]  # Take first detected object
                x_mm, y_mm = best.center_mm

                # Sanity check the mapped coordinates
                if not (MM_X_MIN <= x_mm <= MM_X_MAX and MM_Y_MIN <= y_mm <= MM_Y_MAX):
                    log.warning(f"[SCAN] Mapped coords out of sane range: "
                                f"({x_mm:.1f}, {y_mm:.1f}) mm — skipping")
                    continue

                self.target = PickTarget(
                    label=best.label,
                    x_mm=x_mm,
                    y_mm=y_mm,
                )
                log.info(f"[SCAN] ✓ Target locked: {self.target.label}")
                log.info(f"[SCAN]   pixel=({best.center_px[0]}, {best.center_px[1]})")
                log.info(f"[SCAN]   mm=({self.target.x_mm:.1f}, {self.target.y_mm:.1f})")
                self.state = State.APPROACH
                return

        log.warning(f"[SCAN] Timeout ({SCAN_TIMEOUT_S:.0f}s) — no object marker found")
        self.state = State.IDLE

    async def _state_approach(self):
        """Move above target at approach height."""
        t = self.target
        log.info(f"[APPROACH] Moving to ({t.x_mm:.1f}, {t.y_mm:.1f}, "
                 f"{PICK_Z_APPROACH}) mm  pitch={PICK_PITCH_APPROACH}°")

        # Pre-check reachability with Python IK
        result = ik_solve(t.x_mm, t.y_mm, PICK_Z_APPROACH,
                          pitch_deg=PICK_PITCH_APPROACH,
                          gripper_deg=GRIPPER_OPEN)
        log.info(f"[APPROACH] IK result: {result.msg}")
        log.info(f"[APPROACH] IK angles: {[f'{a:.1f}' for a in result.theta]}")
        if not result.reachable:
            log.warning(f"[APPROACH] Unreachable: {result.msg}")
            self.state = State.HOME
            return

        # Open gripper first
        await self._send_servo(5, GRIPPER_OPEN)

        # Move to approach position
        await self._send_ik(t.x_mm, t.y_mm, PICK_Z_APPROACH,
                            pitch=PICK_PITCH_APPROACH,
                            grip=GRIPPER_OPEN, roll=90.0)
        await asyncio.sleep(MOVE_SETTLE_S)
        self.state = State.PICK

    async def _state_pick(self):
        """Descend, grasp, lift."""
        t = self.target
        log.info(f"[PICK] Descending to ({t.x_mm:.1f}, {t.y_mm:.1f}, "
                 f"{PICK_Z_GRASP}) mm  pitch={PICK_PITCH_GRASP}°")

        # Pre-check grasp position reachability
        result = ik_solve(t.x_mm, t.y_mm, PICK_Z_GRASP,
                          pitch_deg=PICK_PITCH_GRASP,
                          gripper_deg=GRIPPER_OPEN)
        log.info(f"[PICK] IK result: {result.msg}")
        log.info(f"[PICK] IK angles: {[f'{a:.1f}' for a in result.theta]}")
        if not result.reachable:
            log.warning(f"[PICK] Grasp position unreachable: {result.msg}")
            self.state = State.HOME
            return

        # Descend to grasp height
        await self._send_ik(t.x_mm, t.y_mm, PICK_Z_GRASP,
                            pitch=PICK_PITCH_GRASP,
                            grip=GRIPPER_OPEN, roll=90.0)
        await asyncio.sleep(MOVE_SETTLE_S)

        # Close gripper
        log.info("[PICK] Closing gripper")
        await self._send_servo(5, GRIPPER_CLOSED)
        await asyncio.sleep(PICK_PAUSE_S)

        # Lift back to approach height
        log.info("[PICK] Lifting")
        await self._send_ik(t.x_mm, t.y_mm, PICK_Z_APPROACH,
                            pitch=PICK_PITCH_LIFT,
                            grip=GRIPPER_CLOSED, roll=90.0)
        await asyncio.sleep(MOVE_SETTLE_S)
        self.state = State.PLACE

    async def _state_place(self):
        """Move to drop-off and release."""
        log.info(f"[PLACE] Moving to drop-off "
                 f"({PLACE_X}, {PLACE_Y}, {PLACE_Z})  pitch={PLACE_PITCH}°")

        await self._send_ik(PLACE_X, PLACE_Y, PLACE_Z,
                            pitch=PLACE_PITCH,
                            grip=GRIPPER_CLOSED, roll=90.0)
        await asyncio.sleep(MOVE_SETTLE_S)

        # Open gripper to release
        log.info("[PLACE] Releasing")
        await self._send_servo(5, GRIPPER_OPEN)
        await asyncio.sleep(0.5)

        self.state = State.HOME

    async def _state_home(self):
        """Return to home position, then go to IDLE (no loop)."""
        log.info("[HOME] Returning to home position")
        await self.hub.forward_to_esp32(json.dumps({"cmd": "home"}))
        await asyncio.sleep(2.0)
        self.target = None
        self.state = State.IDLE
        log.info("[HOME] Cycle complete — back to IDLE")

    # ── Helpers ───────────────────────────────────────────────

    async def _send_ik(self, x, y, z, pitch=0.0, grip=90.0, roll=90.0):
        """Send an ik_move command to the ESP32 via the hub."""
        cmd = make_ik_command(x, y, z, pitch, grip, roll)
        log.info(f"[CMD] ik_move → x={x:.1f} y={y:.1f} z={z:.1f} "
                 f"pitch={pitch:.1f} grip={grip:.0f} roll={roll:.0f}")
        await self.hub.forward_to_esp32(json.dumps(cmd))

    async def _send_servo(self, ch: int, angle: float):
        """Send a set_servo command to the ESP32 via the hub."""
        cmd = {"cmd": "set_servo", "ch": ch, "angle": angle}
        await self.hub.forward_to_esp32(json.dumps(cmd))

    # ── Public read-only properties for status reporting ─────

    @property
    def state_name(self) -> str:
        return self.state.name

    @property
    def calibrated(self) -> bool:
        return self._calibrated or (self._mapper is not None and self._mapper.is_calibrated)

    @property
    def target_label(self) -> Optional[str]:
        if self.target is not None:
            return self.target.label
        return None

    @property
    def camera_info(self) -> dict:
        """Returns live camera / detection info from the wired VisionEngine."""
        if self._vision is None:
            return {"running": False, "source": None, "detections": 0}
        return {
            "running":    self._vision._running,
            "source":     getattr(self._vision, '_cam_source', None),
            "detections": len(self._vision.latest_detections),
        }

    @property
    def workspace_markers_visible(self) -> list:
        """Return list of workspace marker IDs currently visible."""
        if self._vision is None:
            return []
        return sorted(self._vision.get_workspace_pixel_points().keys())

"""
vision.py — Camera capture loop with ArUco marker detection.

Runs in a background daemon thread. Publishes detection results via asyncio
queue. Exposes MJPEG frames for browser preview via get_jpeg_frame().

Marker roles:
  - IDs 0–3: Workspace calibration corners (known physical positions).
  - IDs 5, 6: Pickable object markers (position computed via homography).
"""

import asyncio
import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Dict, Set, Tuple

import cv2
import numpy as np

log = logging.getLogger("vision")

# ── Camera source ─────────────────────────────────────────────
# DroidCam: install app on phone, set your phone's IP below.
# Default DroidCam port is 4747. Use /video for MJPEG stream.
# To use a local webcam instead, set CAMERA_INDEX = 0
DROIDCAM_IP     = "10.173.117.205"
CAMERA_INDEX    = f"http://{DROIDCAM_IP}:4747/video" if DROIDCAM_IP else 0
FRAME_WIDTH     = 640
FRAME_HEIGHT    = 480
DETECT_INTERVAL = 0.15
WORKSPACE_MARKER_STALE_S = 2.0   # drop workspace markers not seen for this long

# ── Marker roles ──────────────────────────────────────────────
# Workspace corner markers — known physical positions (mm from robot base).
# These are used to compute a homography (pixel → mm).
WORKSPACE_MARKER_MM: Dict[int, tuple] = {
    0: (100.0,  100.0),   # front-left
    1: (300.0,  100.0),   # front-right
    2: (300.0, -100.0),   # back-right
    3: (100.0, -100.0),   # back-left
}

# Object markers — IDs of ArUco markers attached to pickable objects.
# Their mm position is computed via the homography, NOT hardcoded.
OBJECT_MARKER_IDS: Set[int] = {5, 6}


@dataclass
class Detection:
    label: str
    confidence: float
    bbox_px: tuple
    center_px: tuple
    center_mm: Optional[tuple] = None
    marker_id: Optional[int] = None
    role: str = "unknown"       # "workspace", "object", or "unknown"


class VisionEngine:
    def __init__(self, coord_mapper=None):
        self.mapper = coord_mapper
        self.cap = None
        self.latest_frame: Optional[np.ndarray] = None
        self.latest_detections: list[Detection] = []
        self._async_queues: list[asyncio.Queue] = []
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

        # Setup ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # Workspace marker pixel positions — MERGED across frames, with timestamps
        self._workspace_px: Dict[int, tuple] = {}
        self._workspace_px_time: Dict[int, float] = {}   # marker_id → last-seen timestamp

        # Marker role config (mutable via API)
        self._marker_config: Dict[int, Tuple[float, float]] = dict(WORKSPACE_MARKER_MM)

    @property
    def marker_config(self) -> Dict[int, Tuple[float, float]]:
        """Current workspace marker mm positions (readable via /aruco_config)."""
        return dict(self._marker_config)

    def update_marker_config(self, new_config: Dict[int, Tuple[float, float]]):
        """Update workspace marker mm positions at runtime."""
        self._marker_config = dict(new_config)
        log.info(f"Marker config updated: {self._marker_config}")

    def subscribe(self, q: asyncio.Queue):
        self._async_queues.append(q)

    def start(self, loop: asyncio.AbstractEventLoop):
        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        # Only set resolution for local cameras (integer index)
        if isinstance(CAMERA_INDEX, int):
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        if not self.cap.isOpened():
            log.error(f"Cannot open camera: {CAMERA_INDEX}")
            raise RuntimeError(f"Cannot open camera: {CAMERA_INDEX}")

        if isinstance(CAMERA_INDEX, int):
            self._cam_source = f"local webcam ({CAMERA_INDEX})"
        else:
            self._cam_source = f"DroidCam @ {CAMERA_INDEX}"

        self._running = True
        self._thread = threading.Thread(
            target=self._capture_loop, args=(loop,), daemon=True
        )
        self._thread.start()
        log.info(f"VisionEngine started — source: {self._cam_source}")

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3)
        if self.cap:
            self.cap.release()
        log.info("VisionEngine stopped.")

    def get_jpeg_frame(self) -> Optional[bytes]:
        with self._lock:
            frame = self.latest_frame
        if frame is None:
            return None
        _, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        return buf.tobytes()

    def get_workspace_pixel_points(self) -> Dict[int, tuple]:
        """Return the latest pixel centers for workspace markers 0–3."""
        with self._lock:
            return self._workspace_px.copy()

    def _capture_loop(self, loop: asyncio.AbstractEventLoop):
        last_detect = 0.0
        last_detections: list[Detection] = []

        while self._running:
            ret, frame = self.cap.read()
            if not ret:
                log.warning("Frame read failed — skipping")
                time.sleep(0.05)
                continue

            now = time.time()
            if now - last_detect >= DETECT_INTERVAL:
                last_detections = self._run_detection(frame)
                last_detect = now
                self._push_detections(last_detections, loop)

            annotated = self._draw_boxes(frame.copy(), last_detections)

            with self._lock:
                self.latest_frame = annotated
                self.latest_detections = last_detections

            time.sleep(0.01)

    def _run_detection(self, frame: np.ndarray) -> list[Detection]:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        detections = []
        workspace_px_update: Dict[int, tuple] = {}

        if ids is not None:
            for i in range(len(ids)):
                marker_id = int(ids[i][0])
                corner_pts = corners[i][0]

                # Calculate bounding box
                x_coords = corner_pts[:, 0]
                y_coords = corner_pts[:, 1]
                x1, y1 = int(np.min(x_coords)), int(np.min(y_coords))
                x2, y2 = int(np.max(x_coords)), int(np.max(y_coords))

                # Calculate center
                cx = int(np.mean(x_coords))
                cy = int(np.mean(y_coords))

                # ── Determine role and mm coordinates ──
                mm = None
                role = "unknown"

                if marker_id in WORKSPACE_MARKER_MM:
                    # Workspace corner marker — record pixel position for calibration
                    role = "workspace"
                    workspace_px_update[marker_id] = (cx, cy)
                    # mm is the KNOWN position (for display only)
                    mm = WORKSPACE_MARKER_MM[marker_id]

                elif marker_id in OBJECT_MARKER_IDS:
                    # Object marker — compute mm via homography
                    role = "object"
                    if self.mapper and self.mapper.is_calibrated:
                        try:
                            mm = self.mapper.pixel_to_mm(cx, cy)
                        except Exception as e:
                            log.warning(f"pixel_to_mm failed for marker {marker_id}: {e}")

                detections.append(Detection(
                    label=f"Marker {marker_id}",
                    confidence=1.0,
                    bbox_px=(x1, y1, x2, y2),
                    center_px=(cx, cy),
                    center_mm=mm,
                    marker_id=marker_id,
                    role=role,
                ))

        # MERGE workspace pixel positions (don't replace!) and track timestamps
        now = time.time()
        with self._lock:
            # Merge new detections into existing dict
            for mid, px in workspace_px_update.items():
                self._workspace_px[mid] = px
                self._workspace_px_time[mid] = now

            # Expire markers not seen for WORKSPACE_MARKER_STALE_S
            stale_ids = [mid for mid, t in self._workspace_px_time.items()
                         if now - t > WORKSPACE_MARKER_STALE_S]
            for mid in stale_ids:
                self._workspace_px.pop(mid, None)
                self._workspace_px_time.pop(mid, None)
                log.debug(f"Workspace marker {mid} expired (not seen for {WORKSPACE_MARKER_STALE_S}s)")


        return detections

    def _draw_boxes(self, frame: np.ndarray, detections: list[Detection]) -> np.ndarray:
        for d in detections:
            x1, y1, x2, y2 = d.bbox_px

            if d.role == "workspace":
                color = (255, 200, 0)      # Cyan-ish for workspace corners
                label_str = f"WS {d.marker_id}"
                if d.center_mm:
                    label_str += f"  [{d.center_mm[0]:.0f},{d.center_mm[1]:.0f}mm]"
            elif d.role == "object":
                color = (0, 220, 100) if d.center_mm else (0, 165, 255)
                label_str = f"OBJ {d.marker_id}"
                if d.center_mm:
                    label_str += f"  [{d.center_mm[0]:.0f},{d.center_mm[1]:.0f}mm]"
                else:
                    label_str += "  [uncalibrated]"
            else:
                color = (128, 128, 128)
                label_str = f"ID {d.marker_id}"

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label_str, (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.circle(frame, d.center_px, 4, (0, 0, 255), -1)

        # Draw workspace polygon if we have 3+ corners
        ws_pts = []
        with self._lock:
            for mid in sorted(self._workspace_px.keys()):
                ws_pts.append(self._workspace_px[mid])
        if len(ws_pts) >= 3:
            pts = np.array(ws_pts, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], isClosed=True, color=(255, 200, 0), thickness=1)

        return frame

    def _push_detections(self, detections: list[Detection], loop: asyncio.AbstractEventLoop):
        for q in self._async_queues:
            try:
                # Drop oldest if queue is full (prevent back-pressure blocking)
                if q.full():
                    try:
                        q.get_nowait()
                    except asyncio.QueueEmpty:
                        pass
                asyncio.run_coroutine_threadsafe(q.put(detections), loop)
            except Exception as e:
                log.warning(f"Failed to push detections to queue: {e}")

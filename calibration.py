"""
calibration.py — Maps camera pixel coords → robot workspace mm.

Method: Perspective homography (4+ point pairs).
Run once, save to calibration_data.json, reuse forever.

Camera source priority:
  1. DroidCam over Wi-Fi  →  http://<DROIDCAM_IP>:4747/video
  2. Local webcam fallback →  cv2.VideoCapture(0)

Set DROIDCAM_IP below to match your phone's IP (same as vision.py).
Leave it empty ("") to skip DroidCam and always use the local webcam.

Usage:
    python calibration.py   ← interactive calibration tool
"""

import json
import logging
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

log = logging.getLogger("calibration")

CALIB_FILE = Path(__file__).parent / "calibration_data.json"

# ── DroidCam config (must match vision.py) ─────────────────────
# Set to your phone's IP. Leave empty to skip and use local webcam.
DROIDCAM_IP   = "10.173.117.205"
DROIDCAM_PORT = 4747
_DROIDCAM_URL = f"http://{DROIDCAM_IP}:{DROIDCAM_PORT}/video" if DROIDCAM_IP else None

# ── Known physical reference points in robot frame (mm) ────────
# Measure these from the robot's BASE CENTER with a ruler.
# Z is assumed 0 (table surface). Adjust if objects are elevated.
#
# Example layout (tape cross marks on table):
#   P0 = (100,  100)  — front-left
#   P1 = (300,  100)  — front-right
#   P2 = (300, -100)  — back-right
#   P3 = (100, -100)  — back-left
#
# You MUST physically place markers at exactly these positions.
REFERENCE_POINTS_MM = [
    (100.0,  100.0),   # P0 — front-left
    (300.0,  100.0),   # P1 — front-right
    (300.0, -100.0),   # P2 — back-right
    (100.0, -100.0),   # P3 — back-left
]


class CoordMapper:
    """Maps camera pixel coordinates to robot workspace millimetres using
    a perspective homography."""

    def __init__(self):
        self.H: Optional[np.ndarray] = None   # 3×3 homography matrix
        self.is_calibrated: bool = False
        self._try_load()

    def _try_load(self):
        """Try to load saved calibration data on startup."""
        if CALIB_FILE.exists():
            try:
                data = json.loads(CALIB_FILE.read_text())
                self.H = np.array(data["H"], dtype=np.float64)
                self.is_calibrated = True
                log.info(f"Calibration loaded from {CALIB_FILE}")
            except (json.JSONDecodeError, KeyError, ValueError) as e:
                log.warning(f"Failed to load calibration: {e}")
                self.H = None
                self.is_calibrated = False

    def calibrate(self, pixel_points: list, robot_points: list):
        """
        Compute and save a perspective homography from pixel to robot coords.

        pixel_points: [(px, py), ...] — clicked pixel coords (at least 4)
        robot_points: [(x_mm, y_mm), ...] — corresponding robot coords
        """
        assert len(pixel_points) == len(robot_points) >= 4, \
            "Need at least 4 point pairs"

        src = np.array(pixel_points, dtype=np.float32)
        dst = np.array(robot_points, dtype=np.float32)

        # findHomography uses RANSAC — robust to slight measurement error
        self.H, _ = cv2.findHomography(src, dst, cv2.RANSAC, 5.0)

        if self.H is None:
            log.error("Homography computation failed — bad point pairs?")
            self.is_calibrated = False
            return

        self.is_calibrated = True
        self._save()
        log.info("Calibration complete. Homography saved.")

    def pixel_to_mm(self, px: int, py: int) -> tuple[float, float]:
        """Transform a pixel coordinate to robot workspace mm."""
        if not self.is_calibrated or self.H is None:
            raise RuntimeError("Run calibration first!")

        pt = np.array([[[px, py]]], dtype=np.float32)
        result = cv2.perspectiveTransform(pt, self.H)
        x_mm, y_mm = result[0][0]
        return float(x_mm), float(y_mm)

    def calibrate_from_markers(self, workspace_px: dict) -> bool:
        """
        Compute homography from detected workspace marker pixel positions.

        workspace_px: {marker_id: (cx_px, cy_px)} — must contain IDs 0, 1, 2, 3
        Returns True on success, False if insufficient markers.
        """
        required = {0, 1, 2, 3}
        if not required.issubset(workspace_px.keys()):
            missing = required - set(workspace_px.keys())
            log.warning(f"Cannot calibrate — missing workspace markers: {missing}")
            return False

        # Build ordered point pairs: pixel → mm
        pixel_points = []
        robot_points = []
        for mid in sorted(required):
            pixel_points.append(workspace_px[mid])
            robot_points.append(REFERENCE_POINTS_MM[list(sorted(required)).index(mid)])

        self.calibrate(pixel_points, robot_points)
        return self.is_calibrated

    def _save(self):
        """Persist the homography matrix to JSON."""
        data = {"H": self.H.tolist()}
        CALIB_FILE.write_text(json.dumps(data, indent=2))
        log.info(f"Calibration data saved to {CALIB_FILE}")


# ── Interactive calibration tool ─────────────────────────────
def _open_camera() -> cv2.VideoCapture:
    """Open DroidCam first; fall back to local webcam (index 0)."""
    if _DROIDCAM_URL:
        print(f"[CAM] Trying DroidCam at {_DROIDCAM_URL} …")
        cap = cv2.VideoCapture(_DROIDCAM_URL)
        if cap.isOpened():
            print("[CAM] DroidCam connected.")
            return cap
        print("[CAM] DroidCam unreachable — falling back to local webcam.")
        cap.release()

    print("[CAM] Opening local webcam (index 0) …")
    cap = cv2.VideoCapture(0)
    return cap


def run_calibration_tool():
    """
    Interactive OpenCV window. Click the 4 reference markers.
    Press SPACE to compute + save homography. Press Q to quit.

    Camera source: DroidCam (Wi-Fi) → local webcam fallback.
    """
    cap = _open_camera()
    if not cap.isOpened():
        print("ERROR: Cannot open any camera (DroidCam or local).")
        return

    clicks: list[tuple[int, int]] = []

    def on_click(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            idx = len(clicks)
            if idx < len(REFERENCE_POINTS_MM):
                clicks.append((x, y))
                mm = REFERENCE_POINTS_MM[idx]
                print(f"  Point {idx + 1}: pixel ({x},{y})  →  "
                      f"robot ({mm[0]:.0f},{mm[1]:.0f}) mm")
                if len(clicks) == len(REFERENCE_POINTS_MM):
                    print("  All points collected. Press SPACE to calibrate, "
                          "or Q to quit.")

    cv2.namedWindow("Calibration")
    cv2.setMouseCallback("Calibration", on_click)

    print(f"\nCalibration tool — click {len(REFERENCE_POINTS_MM)} "
          f"reference points in order:")
    for i, mm in enumerate(REFERENCE_POINTS_MM):
        print(f"  {i + 1}. Marker at robot ({mm[0]:.0f}, {mm[1]:.0f}) mm")
    print("\nPress SPACE after clicking all points to compute & save.")
    print("Press Q to quit without saving.\n")

    mapper = CoordMapper()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("ERROR: Frame read failed.")
            break

        # Draw clicked points on the frame
        for i, (px, py) in enumerate(clicks):
            cv2.circle(frame, (px, py), 6, (0, 255, 0), -1)
            cv2.putText(frame, f"P{i + 1}", (px + 8, py - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Status bar
        status = f"Click points: {len(clicks)}/{len(REFERENCE_POINTS_MM)}"
        if len(clicks) == len(REFERENCE_POINTS_MM):
            status += "  |  SPACE = calibrate  |  Q = quit"
        cv2.putText(frame, status, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)

        # If calibrated, show live pixel→mm on hover
        if mapper.is_calibrated:
            cv2.putText(frame, "CALIBRATED — click to verify", (10, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 100), 1)

        cv2.imshow("Calibration", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            print("Quitting calibration tool.")
            break
        if key == ord(' ') and len(clicks) == len(REFERENCE_POINTS_MM):
            mapper.calibrate(clicks, REFERENCE_POINTS_MM)
            print("Calibration saved! You can now click to verify, "
                  "or press Q to quit.")
            # Clear clicks for verification mode
            clicks.clear()

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    run_calibration_tool()

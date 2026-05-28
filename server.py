"""
server.py — WebSocket hub for Robotic Arm Controller
------------------------------------------------------
Topology:
  Browser clients  ──┐
                     ├── /ws/browser  (send commands, receive state)
  ESP32 client    ───┘── /ws/esp32   (receive commands, send state)

Install:
  pip install fastapi uvicorn websockets

Run:
  python server.py
  # or for production:
  uvicorn server:app --host 0.0.0.0 --port 8765
"""

import asyncio
import json
import logging
from pathlib import Path
from typing import Optional

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Request
from fastapi.responses import HTMLResponse, FileResponse, StreamingResponse
from vision import VisionEngine
from calibration import CoordMapper
from coordinator import PickPlaceCoordinator

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-8s  %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("arm-server")

app = FastAPI(title="Robotic Arm WS Hub")

# ── Connection registry ───────────────────────────────────────

class ConnectionHub:
    def __init__(self):
        self.browsers: set[WebSocket] = set()
        self.esp32: Optional[WebSocket] = None
        self._lock = asyncio.Lock()

    async def add_browser(self, ws: WebSocket):
        async with self._lock:
            self.browsers.add(ws)
        log.info(f"Browser connected  (total: {len(self.browsers)})")

    async def remove_browser(self, ws: WebSocket):
        async with self._lock:
            self.browsers.discard(ws)
        log.info(f"Browser disconnected  (total: {len(self.browsers)})")

    async def set_esp32(self, ws: WebSocket):
        async with self._lock:
            if self.esp32 is not None:
                log.warning("ESP32 re-connected — replacing previous socket")
            self.esp32 = ws
        log.info("ESP32 connected")

    async def remove_esp32(self, ws: WebSocket):
        async with self._lock:
            if self.esp32 is ws:
                self.esp32 = None
        log.info("ESP32 disconnected")

    async def forward_to_esp32(self, message: str) -> bool:
        """Send a command from a browser to the ESP32. Returns True on success."""
        async with self._lock:
            esp = self.esp32
        if esp is None:
            log.warning("No ESP32 connected — command dropped")
            return False
        try:
            await esp.send_text(message)
            return True
        except Exception as e:
            log.error(f"ESP32 send error: {e}")
            return False

    async def broadcast_to_browsers(self, message: str):
        """Relay ESP32 state updates to all browser clients."""
        async with self._lock:
            targets = list(self.browsers)
        dead: list[WebSocket] = []
        for ws in targets:
            try:
                await ws.send_text(message)
            except Exception:
                dead.append(ws)
        for ws in dead:
            await self.remove_browser(ws)

    @property
    def esp32_connected(self) -> bool:
        return self.esp32 is not None

    def status(self) -> dict:
        return {
            "esp32_connected": self.esp32_connected,
            "browser_count": len(self.browsers),
        }


hub = ConnectionHub()

# ── Vision, calibration, and coordinator globals ─────────────
mapper      = CoordMapper()
vision      = VisionEngine(coord_mapper=mapper)
coordinator = PickPlaceCoordinator(hub, mapper=mapper, vision=vision)
vision.subscribe(coordinator.get_detection_queue())

# ── Allowed commands the server will forward to ESP32 ────────
ALLOWED_COMMANDS = {"set_servo", "estop", "home", "ik_move", "record_start", "record_stop", "playback_start", "playback_stop", "preset_goto", "preset_save"}

# ─────────────────────────────────────────────────────────────
#  Browser WebSocket endpoint
# ─────────────────────────────────────────────────────────────
@app.websocket("/ws/browser")
async def browser_endpoint(ws: WebSocket):
    await ws.accept()
    await hub.add_browser(ws)

    # Send immediate status so the UI knows if ESP32 is present
    await ws.send_text(json.dumps({"type": "server_status", **hub.status()}))

    try:
        while True:
            raw = await ws.receive_text()

            # Validate JSON and filter allowed commands
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                await ws.send_text(json.dumps({"type": "error", "msg": "invalid JSON"}))
                continue

            cmd = msg.get("cmd", "")

            # Handle calibrate command locally
            if cmd == "calibrate":
                ok = coordinator.start_calibration()
                await ws.send_text(json.dumps({
                    "type": "calibrate_status",
                    "started": ok,
                    "state": coordinator.state_name,
                }))
                continue

            # Handle scan command locally
            if cmd == "scan":
                ok = coordinator.start_scan()
                await ws.send_text(json.dumps({
                    "type": "scan_status",
                    "started": ok,
                    "state": coordinator.state_name,
                    "calibrated": coordinator.calibrated,
                }))
                continue

            if cmd == "toggle_camera":
                enable = bool(msg.get("enable", False))
                loop = asyncio.get_running_loop()
                if enable and not vision._running:
                    try:
                        vision.start(loop)
                    except Exception as e:
                        log.warning(f"Vision failed to start: {e}")
                elif not enable and vision._running:
                    vision.stop()
                continue

            if cmd not in ALLOWED_COMMANDS:
                await ws.send_text(json.dumps({"type": "error", "msg": f"unknown cmd: {cmd}"}))
                continue

            ok = await hub.forward_to_esp32(raw)
            if not ok:
                await ws.send_text(json.dumps({"type": "error", "msg": "ESP32 not connected"}))

    except WebSocketDisconnect:
        pass
    finally:
        await hub.remove_browser(ws)


# ─────────────────────────────────────────────────────────────
#  ESP32 WebSocket endpoint
# ─────────────────────────────────────────────────────────────
@app.websocket("/ws/esp32")
async def esp32_endpoint(ws: WebSocket):
    await ws.accept()
    await hub.set_esp32(ws)

    # Notify all browsers that ESP32 is online
    await hub.broadcast_to_browsers(
        json.dumps({"type": "server_status", **hub.status()})
    )

    try:
        while True:
            raw = await ws.receive_text()
            # ESP32 sends state updates → relay to browsers
            try:
                msg = json.loads(raw)
                msg["type"] = "esp32_state"           # tag the source
                await hub.broadcast_to_browsers(json.dumps(msg))
            except json.JSONDecodeError:
                log.warning(f"ESP32 sent invalid JSON: {raw[:80]}")

    except WebSocketDisconnect:
        pass
    finally:
        await hub.remove_esp32(ws)
        await hub.broadcast_to_browsers(
            json.dumps({"type": "server_status", **hub.status()})
        )


# ── Health check ─────────────────────────────────────────────
@app.get("/status")
async def status():
    return hub.status()


# ── Serve dashboard files ────────────────────────────────────
_STATIC_DIR = Path(__file__).parent

@app.get("/", response_class=HTMLResponse)
@app.get("/index copy.html", response_class=HTMLResponse)
@app.get("/index.html", response_class=HTMLResponse)
async def serve_dashboard():
    html_path = _STATIC_DIR / "index.html"
    if html_path.exists():
        return HTMLResponse(content=html_path.read_text(encoding="utf-8"))
    return HTMLResponse(content="<h1>Dashboard not found</h1>", status_code=404)

@app.get("/dashboard.css")
async def serve_css():
    return FileResponse(
        _STATIC_DIR / "dashboard.css",
        media_type="text/css",
        headers={"Cache-Control": "no-cache, no-store, must-revalidate", "Pragma": "no-cache"},
    )

@app.get("/dashboard.js")
async def serve_js():
    return FileResponse(
        _STATIC_DIR / "dashboard.js",
        media_type="application/javascript",
        headers={"Cache-Control": "no-cache, no-store, must-revalidate", "Pragma": "no-cache"},
    )


# ── Vision status endpoint ───────────────────────────────────
@app.get("/vision_status")
async def vision_status():
    dets = []
    with vision._lock:
        for d in vision.latest_detections:
            dets.append({
                "label": d.label,
                "marker_id": d.marker_id,
                "center_mm": d.center_mm,
                "role": getattr(d, 'role', 'unknown'),
            })
    return {
        "calibrated":       coordinator.calibrated,
        "arm_state":        coordinator.state_name,
        "target":           coordinator.target_label,
        "camera_running":   vision._running,
        "detections":       dets,
        "workspace_markers": coordinator.workspace_markers_visible,
    }

# ── ArUco Configuration ──────────────────────────────────────
@app.get("/aruco_config")
async def get_aruco_config():
    return vision.marker_config

@app.post("/aruco_config")
async def set_aruco_config(request: Request):
    try:
        data = await request.json()
        new_config = {int(k): tuple(v) for k, v in data.items()}
        vision.update_marker_config(new_config)
        return {"status": "success", "config": vision.marker_config}
    except Exception as e:
        return {"status": "error", "msg": str(e)}


# ── MJPEG video stream ───────────────────────────────────────
@app.get("/video_feed")
async def video_feed():
    async def generate():
        while True:
            frame = vision.get_jpeg_frame()
            if frame:
                yield (b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"
                       + frame + b"\r\n")
            await asyncio.sleep(0.05)   # ~20fps
    return StreamingResponse(
        generate(),
        media_type="multipart/x-mixed-replace; boundary=frame"
    )


# ── Startup event ────────────────────────────────────────────
@app.on_event("startup")
async def on_startup():
    loop = asyncio.get_event_loop()
    try:
        vision.start(loop)
    except RuntimeError as e:
        log.warning(f"VisionEngine failed to start: {e} — server continues without camera")
    asyncio.create_task(coordinator.run())


# ─────────────────────────────────────────────────────────────
if __name__ == "__main__":
    uvicorn.run(
        "server:app",
        host="0.0.0.0",
        port=8765,
        log_level="info",
        ws_ping_interval=20,
        ws_ping_timeout=10,
    )

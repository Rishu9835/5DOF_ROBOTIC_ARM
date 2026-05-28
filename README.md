# Robotic Arm — Refactored WS-Client Architecture

## System Overview

```
Browser (index.html)
    │  WebSocket /ws/browser
    ▼
Python Server (server.py)  ← FastAPI + uvicorn
    │  WebSocket /ws/esp32
    ▼
ESP32 (main.ino)            ← WebSocketsClient (STA mode only)
    │  I2C
    ▼
PCA9685 → 6× Servos
```

No web server runs on the ESP32. The ESP32 only connects outward
to the Python backend. This eliminates the AsyncWebServer heap
pressure that caused SW_CPU_RESET crashes.

---

## Quick Start

### 1. Backend (Python)

```bash
pip install fastapi uvicorn websockets
python server.py
# Listening on ws://0.0.0.0:8765
```

### 2. Frontend

Open `index.html` in a browser.
- If served via file://, change `WS_URL` in the script to your server IP:
  `const WS_URL = 'ws://192.168.1.100:8765/ws/browser';`
- Or serve it with the backend — add a StaticFiles mount to server.py.

### 3. ESP32

1. Set your WiFi credentials in `main.ino`:
   ```cpp
   constexpr char WIFI_SSID[]     = "YourNetwork";
   constexpr char WIFI_PASSWORD[] = "YourPassword";
   ```

2. Set your server IP:
   ```cpp
   constexpr char WS_HOST[] = "192.168.1.100";  // ← your machine's LAN IP
   ```

3. Flash with PlatformIO or Arduino IDE (see `platformio.ini` for libraries).

---

## Supported Commands (JSON)

| Command    | Payload                                              | Description         |
|------------|------------------------------------------------------|---------------------|
| set_servo  | `{"cmd":"set_servo","ch":0,"angle":90}`              | Move one joint      |
| estop      | `{"cmd":"estop"}`                                    | Emergency stop      |
| home       | `{"cmd":"home"}`                                     | All joints to home  |
| ik_move    | `{"cmd":"ik_move","x":200,"y":0,"z":150,"pitch":0,"grip":90,"roll":90}` | Cartesian move |

---

## Files

| File               | Role                                      |
|--------------------|-------------------------------------------|
| `main.ino`         | ESP32 firmware — WS client only           |
| `servo_controller.h` | PCA9685 servo driver (unchanged)        |
| `ik_solver.h`      | Geometric IK solver (unchanged)           |
| `recording_engine.h` | Keyframe recorder/playback (unchanged)  |
| `server.py`        | Python WS hub — routes commands to ESP32  |
| `index.html`       | Browser control panel                     |
| `platformio.ini`   | PlatformIO build config                   |

---

## Memory Notes

- `StaticJsonDocument<256>` — fixed on stack, no heap allocation per message.
- No `String` objects in the hot path.
- AsyncTCP + AsyncWebServer removed entirely — saves ~40KB heap.
- Bluetooth disabled — saves ~60KB heap.
- Expected free heap at runtime: **~180–220KB** (up from ~60KB before).

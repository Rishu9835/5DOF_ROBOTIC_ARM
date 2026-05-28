# 🦾 5-DOF Robotic Arm — Autonomous Pick & Place
<img width="461" height="596" alt="Screenshot 2026-05-29 at 12 49 41 AM" src="https://github.com/user-attachments/assets/027c97af-ffac-4aba-ab59-4211baf71b28" />
<img width="620" height="457" alt="Screenshot 2026-05-29 at 12 50 33 AM" src="https://github.com/user-attachments/assets/8a42fe57-9571-46dd-a863-a9f0a85f6841" />


> ESP32-powered robotic arm with ArUco vision, geometric IK solver, and a real-time web dashboard.

---

## 📋 Table of Contents

- [Overview](#-overview)
- [System Architecture](#-system-architecture)
- [Hardware](#-hardware)
- [Kinematics](#-kinematics)
- [Software Components](#-software-components)
  - [ESP32 Firmware](#1-esp32-firmware)
  - [Python Backend Server](#2-python-backend-server)
  - [Vision Engine](#3-vision-engine-visionpy)
  - [Pick-and-Place Coordinator](#4-pick-and-place-coordinator-coordinatorpy)
  - [Web Dashboard](#5-web-dashboard)
- [Getting Started](#-getting-started)
  - [Prerequisites](#prerequisites)
  - [Firmware Setup](#firmware-setup-platformio)
  - [Backend Setup](#backend-setup-python)
  - [Running the System](#running-the-system)
- [Autonomous Pick & Place Workflow](#-autonomous-pick--place-workflow)
- [Manual Dashboard Controls](#-manual-dashboard-controls)
- [File Structure](#-file-structure)
- [Configuration Reference](#-configuration-reference)
- [Wiring](#-wiring)
- [Troubleshooting](#-troubleshooting)
- [Author](#-author)
- [License](#-license)

---

## 🔭 Overview

This project implements a **5-DOF robotic arm** capable of both manual joint-level control and fully autonomous **pick-and-place** using a camera-based ArUco marker vision system.

**Key capabilities:**
- ✅ Geometric Inverse Kinematics (IK) solver running on the ESP32
- ✅ Autonomous pick-and-place driven by ArUco marker detection
- ✅ Real-time web dashboard with 3D arm visualization
- ✅ Smooth servo interpolation (no sudden jumps)
- ✅ Motion recording & playback
- ✅ Named pose presets
- ✅ Emergency stop

**Communication flow:**

```
Browser (Dashboard) ──WebSocket──► Python Server ──WebSocket──► ESP32
                                        │
                                   Camera Feed (DroidCam / Webcam)
                                        │
                                   ArUco Detection ──► Coordinator ──► ESP32 IK Commands
```

---

## 🏗 System Architecture

The system uses a **3-tier architecture**:

| Tier | Component | Role |
|------|-----------|------|
| **Edge** | ESP32 + PCA9685 | Servo control, IK computation, WS client |
| **Middle** | Python FastAPI server | WebSocket hub, vision processing, coordination |
| **Frontend** | Browser dashboard | Real-time control, 3D visualization, camera preview |

The ESP32 acts as a pure **WebSocket client** — it connects outward to the Python server. This design avoids running a heavy WebSocket server on the ESP32, significantly reducing heap usage and improving stability.

---

## 🔩 Hardware

| Component | Details |
|-----------|---------|
| **Microcontroller** | ESP32 Dev Module (240 MHz, dual-core) |
| **Servo Driver** | PCA9685 16-channel PWM driver (I2C @ 0x40) |
| **Servos** | 6× SG90 / MG90S |
| **Camera** | DroidCam (phone) or USB webcam |
| **Power** | 5V for servos (separate from ESP32 logic) |

### Joint Mapping (PCA9685 Channels)

| Channel | Joint | Range | Notes |
|---------|-------|-------|-------|
| 0 | Base (waist) | 0°–180° | Rotates around Z-axis |
| 1 | Shoulder | 0°–90° | Upper arm pitch |
| 2 | Elbow | 0°–90° | Forearm pitch |
| 3 | Wrist Pitch | 0°–150° | + mirror servo on ch6 |
| 4 | Wrist Roll | 0°–180° | Arm_03 roll (orientation only) |
| 5 | Gripper | 80°–180° | 80=closed, 180=open |
| 6 | Wrist Mirror | — | Auto-driven (inverse of ch3) |

---

## 📐 Kinematics

### Link Lengths (from STL geometry)

| Segment | Length | Role |
|---------|--------|------|
| Base height | 56.0 mm | Vertical offset from floor to shoulder pivot |
| L1 (Arm_01) | 166.7 mm | Upper arm (pitch) |
| L2 (Arm_02) | 115.0 mm | Forearm (pitch) |
| L3 (Arm_03) | 46.0 mm | **Roll joint** — merged into effective forearm |
| L4 (GripperBase) | 77.0 mm | Wrist offset (pitch) |
| L5 (Gripper tip) | 67.3 mm | End-effector offset |

### IK Model

The solver uses a **2-link planar IK** with the following derived constants:

```
IK_L2_EFF = L2 + L3 = 161.0 mm  (effective forearm — L3 is roll, not pitch)
IK_WRIST  = L4 + L5 = 144.3 mm  (wrist chain offset)
```

**Algorithm (in `ik_solver.h`):**
1. Compute base angle from `(x, y)` position
2. Subtract wrist offset from target to get elbow-chain endpoint
3. Law of cosines for shoulder + elbow angles
4. **Clamp** shoulder/elbow to servo limits
5. **Recompute wrist pitch from the clamped angles** (critical fix — prevents wrist errors near joint limits)
6. Pass roll and gripper through unchanged (orientation-only joints)

**Coordinate system:** X forward, Y left, Z up. 0° base = arm pointing forward (+X).

---

## 💻 Software Components

### 1. ESP32 Firmware

**File:** `ROBOTIC_ARM_FINAL.ino`

The firmware is a pure **WebSocket client** built on the Arduino framework. It:
- Connects to WiFi and establishes a WS connection to `ws://<SERVER_IP>:8765/ws/esp32`
- Announces itself with `{"role":"esp32"}` on connect
- Processes JSON commands dispatched from the Python server
- Drives servos via `ServoController` (smooth interpolation at 50 Hz)
- Heartbeat: ping every 15s, timeout after 3s, disconnect after 2 missed pings

**Supported commands:**

| Command | Params | Description |
|---------|--------|-------------|
| `set_servo` | `ch`, `angle` | Move a single servo |
| `ik_move` | `x`, `y`, `z`, `pitch`, `grip`, `roll` | Move to Cartesian position |
| `home` | — | Return all joints to home position |
| `estop` | — | Emergency stop (freeze all joints) |
| `record_start` | — | Begin recording motion |
| `record_stop` | — | Stop recording |
| `playback_start` | `speed`, `loop` | Play back recorded motion |
| `playback_stop` | — | Stop playback |
| `preset_save` | `name`, `angles[]` | Save named pose |
| `preset_goto` | `name` | Move to named pose |

---

### 2. Python Backend Server

**File:** `server.py`

Built with **FastAPI + uvicorn**, it serves as the central WebSocket hub and HTTP server.

**Endpoints:**

| Endpoint | Type | Description |
|----------|------|-------------|
| `/` | HTTP GET | Serves the web dashboard (`index.html`) |
| `/dashboard.css` | HTTP GET | Dashboard styles |
| `/dashboard.js` | HTTP GET | Dashboard logic |
| `/ws/browser` | WebSocket | Browser ↔ server connection |
| `/ws/esp32` | WebSocket | ESP32 ↔ server connection |
| `/status` | HTTP GET | JSON connection status |
| `/vision_status` | HTTP GET | Camera + detection info |
| `/video_feed` | HTTP GET | MJPEG camera stream |
| `/aruco_config` | HTTP GET/POST | Read/update marker mm positions |

**Connection model:**
- Multiple browser clients can connect simultaneously
- Only one ESP32 client is tracked at a time (replaced on reconnect)
- Commands from the browser are validated against an allowlist before forwarding to the ESP32

---

### 3. Vision Engine (`vision.py`)

Runs in a **background daemon thread**. Captures frames from DroidCam or a webcam, detects ArUco markers, and pushes detection results into an asyncio queue for the coordinator.

**Marker roles:**

| Marker IDs | Role | Position |
|------------|------|----------|
| 0, 1, 2, 3 | Workspace corners | Known physical mm positions |
| 5, 6 | Pickable objects | Computed via homography |

**Key features:**
- Uses OpenCV `DICT_5X5_50` ArUco dictionary
- Workspace marker pixel positions are **merged across frames** with a 2-second staleness window (robust to momentary occlusion)
- Homography (pixel → mm) computed from 4 corner markers
- Annotated MJPEG stream served for real-time browser preview

---

### 4. Pick-and-Place Coordinator (`coordinator.py`)

User-triggered **async state machine** that orchestrates the full pick-and-place cycle.

**State machine:**

```
IDLE ──[CALIBRATE]──► CALIBRATE ──► IDLE (calibrated)

IDLE ──[SCAN]──► SCAN ──► APPROACH ──► PICK ──► PLACE ──► HOME ──► IDLE
```

**Pick-and-place sequence:**
1. **SCAN** — Wait for object marker (5 or 6) with valid mm coordinates
2. **APPROACH** — Open gripper, move to hover height (`z = 100 mm`) above object
3. **PICK** — Descend to grasp height (`z = 50 mm`), close gripper, lift back up
4. **PLACE** — Move to fixed drop-off position `(250, 0, 80) mm`, release gripper
5. **HOME** — Return to home position → IDLE

**Physical parameters (configurable at top of `coordinator.py`):**

```python
PICK_Z_APPROACH  = 100.0 mm     # hover height
PICK_Z_GRASP     =  50.0 mm     # grasp height
PICK_PITCH_GRASP = -80.0°       # nearly vertical wrist for low Z
PLACE_X/Y/Z      = 250, 0, 80   # drop-off position
GRIPPER_OPEN     = 170°
GRIPPER_CLOSED   =  20°
```

---

### 5. Web Dashboard

**Files:** `index.html`, `dashboard.css`, `dashboard.js`

A full-featured browser UI served by the Python backend:

- **Live 3D arm visualization** — WebGL-based viewport that mirrors real-time joint angles
- **Manual controls** — Sliders + numeric inputs for each joint with bidirectional sync
- **IK input mode** — Enter target `(x, y, z)` coordinates directly; IK is solved server-side
- **Autonomous mode** — CALIBRATE and SCAN buttons for pick-and-place workflow
- **Camera preview** — Embedded MJPEG stream from the vision engine
- **Recording panel** — Record, stop, and play back arm motions with speed/loop controls
- **Presets** — Save and restore named poses
- **E-Stop button** — Immediately freezes all joints
- **Connection status** — Live indicator for ESP32 connectivity

---

## 🚀 Getting Started

### Prerequisites

**Hardware:**
- ESP32 Dev Module
- PCA9685 servo driver
- 6× SG90 or MG90S servos
- DroidCam app (or USB webcam)
- 4× ArUco markers (IDs 0–3) for workspace calibration
- 1–2× ArUco markers (IDs 5–6) attached to pick objects

**Software:**
- Python 3.10+
- PlatformIO (CLI or VSCode extension)
- Arduino IDE (alternative to PlatformIO)
- OpenCV (installed via `requirements.txt`)

---

### Firmware Setup (PlatformIO)

1. **Clone the repo:**
   ```bash
   git clone https://github.com/Rishu9835/5DOF_ROBOTIC_ARM.git
   cd 5DOF_ROBOTIC_ARM
   ```

2. **Edit WiFi credentials and server IP** in `ROBOTIC_ARM_FINAL.ino`:
   ```cpp
   constexpr char WIFI_SSID[]     = "YourWiFiName";
   constexpr char WIFI_PASSWORD[] = "YourWiFiPassword";
   constexpr char WS_HOST[]       = "192.168.x.x";  // ← your PC's IP
   ```

3. **Flash via PlatformIO:**
   ```bash
   pio run --target upload
   pio device monitor   # view serial output
   ```

   > **Arduino IDE users:** Install the three libraries listed in `platformio.ini` via Library Manager. Set board to **ESP32 Dev Module**, CPU Frequency **240MHz**.

4. **Calibrate PWM pulse widths** in `servo_controller.h` if your servos differ from the defaults (102–512 ticks):
   ```cpp
   constexpr uint16_t SERVO_MIN[NUM_SERVOS] = {102, 102, 102, 102, 102, 102};
   constexpr uint16_t SERVO_MAX[NUM_SERVOS] = {512, 512, 512, 512, 512, 512};
   ```

---

### Backend Setup (Python)

1. **Create a virtual environment and install dependencies:**
   ```bash
   python3 -m venv venv
   source venv/bin/activate       # Windows: venv\Scripts\activate
   pip install -r requirements.txt
   ```

2. **Configure the camera source** in `vision.py`:
   ```python
   DROIDCAM_IP  = "192.168.x.x"   # your phone's IP (DroidCam)
   # — or for a USB webcam —
   DROIDCAM_IP  = ""               # empty string → uses CAMERA_INDEX = 0
   ```

3. **Set workspace marker physical positions** in `vision.py` (default values in mm from robot base):
   ```python
   WORKSPACE_MARKER_MM = {
       0: (100.0,  100.0),   # front-left
       1: (300.0,  100.0),   # front-right
       2: (300.0, -100.0),   # back-right
       3: (100.0, -100.0),   # back-left
   }
   ```

---

### Running the System

1. **Start the Python server:**
   ```bash
   python server.py
   # or
   uvicorn server:app --host 0.0.0.0 --port 8765
   ```

2. **Power on the ESP32** — it will auto-connect to WiFi and the WebSocket server. Watch the serial monitor for:
   ```
   [WiFi] Connected. IP: 192.168.x.x
   [WS] Connected to /ws/esp32
   ```

3. **Open the dashboard** in your browser:
   ```
   http://<your-pc-ip>:8765
   ```

4. The ESP32 status indicator in the dashboard should turn **green** once connected.

---

## 🤖 Autonomous Pick & Place Workflow

### Setup
1. Print and place **4 ArUco markers** (IDs 0–3) flat at the corners of your workspace. Measure their exact positions from the robot base and update `WORKSPACE_MARKER_MM` in `vision.py`.
2. Point the camera so all 4 corner markers are visible.

### Operation
1. **CALIBRATE** — Click the Calibrate button in the dashboard. The system waits up to 10 seconds for all 4 markers to be simultaneously visible, then computes a pixel→mm homography.
2. **Place object** — Put an item with ArUco marker ID **5** or **6** anywhere in the workspace.
3. **SCAN** — Click the Scan button. The coordinator detects the object, maps its position to mm, and executes the pick-and-place sequence automatically.
4. The arm **returns to home** after each successful cycle.

> **Tip:** If calibration fails, check the serial/server logs. The system will report which markers (0–3) it could not see.

---

## 🕹 Manual Dashboard Controls

| Control | Description |
|---------|-------------|
| Joint sliders | Move each servo individually with live feedback |
| Numeric inputs | Precise angle entry (bidirectionally synced with sliders) |
| IK Mode | Enter `(x, y, z)` target; arm moves using IK |
| Home button | Return all joints to home position |
| E-Stop | Freeze arm immediately |
| Record | Capture a sequence of movements |
| Playback | Replay recorded sequence (with speed multiplier + loop option) |
| Presets | Save/load named poses |

---

## 📁 File Structure

```
5DOF_ROBOTIC_ARM/
│
├── ROBOTIC_ARM_FINAL.ino   # ESP32 main firmware (WiFi, WS client, command dispatcher)
├── ik_solver.h             # Geometric IK + FK solver (header-only, runs on ESP32)
├── servo_controller.h      # PCA9685 servo driver with interpolation & presets
├── recording_engine.h      # Motion recording & playback engine
├── wifi_manager.h          # WiFi connection helper
│
├── server.py               # FastAPI WebSocket hub + HTTP server
├── vision.py               # ArUco marker detection (background thread)
├── coordinator.py          # Async pick-and-place state machine
├── calibration.py          # Homography calibration (pixel → mm mapping)
├── ik_bridge.py            # Python mirror of the IK solver (for pre-checking reachability)
│
├── index.html              # Web dashboard HTML
├── dashboard.css           # Dashboard styles
├── dashboard.js            # Dashboard logic (3D visualization, WS client)
│
├── requirements.txt        # Python dependencies
├── platformio.ini          # PlatformIO build configuration
└── calibration_data.json   # Saved homography matrix (auto-generated)
```

---

## ⚙ Configuration Reference

### `servo_controller.h`

| Constant | Default | Description |
|----------|---------|-------------|
| `SERVO_MIN[6]` | `{102, …}` | PCA9685 ticks at 0° per channel |
| `SERVO_MAX[6]` | `{512, …}` | PCA9685 ticks at 180° per channel |
| `ANGLE_MIN[6]` | `{0,0,0,0,0,80}` | Min angle per joint |
| `ANGLE_MAX[6]` | `{180,90,90,150,180,180}` | Max angle per joint |
| `HOME_ANGLES[6]` | `{90,90,45,43,110,80}` | Home pose |
| `INTERP_STEP` | `1.0°/tick` | Interpolation speed (50Hz → 50°/s) |
| `DEADBAND` | `2.0°` | Stop threshold to prevent micro-oscillations |

### `coordinator.py`

| Constant | Default | Description |
|----------|---------|-------------|
| `PICK_Z_APPROACH` | `100.0 mm` | Hover height during approach |
| `PICK_Z_GRASP` | `50.0 mm` | Descent height for grasping |
| `PICK_PITCH_APPROACH` | `-60°` | Wrist tilt during approach |
| `PICK_PITCH_GRASP` | `-80°` | Wrist tilt at grasp |
| `PLACE_X/Y/Z` | `250, 0, 80 mm` | Fixed drop-off position |
| `GRIPPER_OPEN` | `170°` | Open position |
| `GRIPPER_CLOSED` | `20°` | Grasping position |
| `MOVE_SETTLE_S` | `1.2 s` | Wait time after each IK move |
| `SCAN_TIMEOUT_S` | `15.0 s` | How long to wait for object detection |

---

## 🔌 Wiring

```
ESP32 ──I2C──► PCA9685 (0x40)
               │
               ├── CH0 → Base servo
               ├── CH1 → Shoulder servo
               ├── CH2 → Elbow servo
               ├── CH3 → Wrist pitch servo (primary)
               ├── CH4 → Wrist roll servo
               ├── CH5 → Gripper servo
               └── CH6 → Wrist pitch mirror servo (inverted, optional)

ESP32 SDA → GPIO21
ESP32 SCL → GPIO22
PCA9685 VCC → 3.3V (logic)
PCA9685 V+ → 5V (servo power — use separate supply!)
```

> ⚠️ **Never power servos from the ESP32's 3.3V/5V pins**. Use a dedicated 5V supply rated for your servo count (at least 2A for 6× SG90).

---

## 🛠 Troubleshooting

| Issue | Likely Cause | Fix |
|-------|-------------|-----|
| ESP32 won't connect to WS | Wrong server IP or port | Check `WS_HOST` in `.ino` and ensure the Python server is running |
| Dashboard shows "ESP32 Offline" | ESP32 hasn't connected yet | Watch serial monitor; check WiFi credentials |
| Arm moves to wrong position | PWM pulse range mismatch | Calibrate `SERVO_MIN/MAX` per channel |
| IK returns "Unreachable" | Target outside workspace | Reduce X reach, increase Z, or check that pitch doesn't over-extend the chain |
| Calibration timeout | Not all 4 markers visible | Improve lighting, ensure markers 0–3 are all in camera frame |
| Camera feed not showing | Wrong DroidCam IP or phone on different network | Update `DROIDCAM_IP` in `vision.py`; ensure phone and PC are on same WiFi |
| Gripper doesn't hold | PWM silenced intentionally | This is by design — friction holds the grip. If insufficient, increase `GRIPPER_CLOSED` angle |

---

## 👤 Author

**Rishu Raj**
- GitHub: [@Rishu9835](https://github.com/Rishu9835)
- Email: rishuraj9431@gmail.com

---

## 📄 License

This project is licensed under the **MIT License** — see the [LICENSE](LICENSE) file for details.

```
MIT License

Copyright (c) 2026 Rishu Raj

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

*Built with ❤️ using ESP32, FastAPI, OpenCV, and WebSockets.*

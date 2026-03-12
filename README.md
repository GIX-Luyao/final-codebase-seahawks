# Seahawk

A hybrid drone-assisted system for deploying and retrieving ocean monitoring devices in coastal field environments.

---

## 1. Project Overview

Seahawk is a capstone project developed to address the challenge of safely deploying and retrieving sensor devices in hazardous coastal areas. The system eliminates the need for researchers to physically enter dangerous water environments by leveraging an autonomous drone platform equipped with computer vision, a winch mechanism, and a servo-driven gripper.

**Problem Statement:**
Ocean researchers often need to place or recover monitoring instruments in locations that are difficult or dangerous to access on foot. Manual deployment exposes personnel to tidal surges, slippery terrain, and other coastal hazards.

**Target Users:**
Environmental researchers, marine scientists, and field engineers who require a remotely operated system for instrument deployment and retrieval.

**Core Capabilities:**
- Real-time visual target detection via HSV color-based tracking and YOLOv8
- Manual and semi-autonomous drone flight control with keyboard or web interface
- Coordinated winch lowering and gripper actuation for payload retrieval
- Web-based operator interface for monitoring, manual flight, and system control
- LoRa-based remote GPS coordinate relay from a ground station
- GPS waypoint navigation to a target location

---

## 2. Repository Structure

```
seahawk/
│
├── gui/
│   ├── drone_control_gui.py        # Main application: Flask web GUI
│   ├── test_gui_demo.py            # Demo mode (runs without drone hardware)
│   ├── start_gui.sh                # Launch script
│   ├── menu.sh                     # Interactive launch menu
│   ├── check_dependencies.py       # Dependency verification utility
│   ├── requirements_gui.txt        # Python dependencies for the GUI
│   ├── README.md                   # GUI-specific documentation
│   ├── templates/
│   │   └── index.html              # Vue.js frontend template
│   └── static/
│       ├── css/style.css           # UI stylesheet
│       ├── js/app.js               # Vue.js application logic
│       ├── js/vue.global.js        # Vue.js runtime
│       └── img/                    # UI assets (background, logo)
│
├── fly_watch_stream_ai_keyboard.py # Terminal-based flight + live video + keyboard control
├── test_color_tracking.py          # HSV color tracking with drone flight control
├── requirements.txt                # Python dependencies (perception / core)
├── .gitignore
├── README.md
└── user_manual.pdf                 # PDF user manual
```

| Item | Description |
|------|-------------|
| `gui/` | Flask + Vue.js web control interface integrating navigation, perception, and winch/gripper subsystems into a unified dashboard |
| `fly_watch_stream_ai_keyboard.py` | Standalone terminal script for manual drone piloting with real-time OpenCV video display, gimbal control, and keyboard commands |
| `test_color_tracking.py` | HSV color-based target tracking script with three operating modes (calibrate, track, webcam) and full flight control |
| `requirements.txt` | Core Python dependencies (OpenCV, Ultralytics YOLOv8, NumPy, PyTorch) |

---

## 3. System Requirements

### Software

| Component | Requirement |
|-----------|-------------|
| Operating System | Ubuntu 20.04+ or Debian-based Linux |
| Python | 3.8 or higher |
| Parrot Olympe SDK | 7.x (installed separately per Parrot documentation) |
| OpenCV | 4.5.0+ |
| Ultralytics (YOLOv8) | 8.0.0+ |
| NumPy | 1.20.0+ |
| Flask | 3.0.0 |
| Flask-SocketIO | 5.3.5 |
| Web Browser | Chrome, Firefox, or Edge (for GUI access) |

Optional: NVIDIA GPU with CUDA 11.8+ for accelerated YOLOv8 inference.

### Hardware

| Component | Description |
|-----------|-------------|
| Drone Platform | Parrot ANAFI Ai (Wi-Fi IP: 192.168.42.1) |
| Onboard Camera | ANAFI Ai integrated camera (used for perception stream) |
| Winch Controller | ESP32 microcontroller with continuous-rotation servo, hall-effect sensor, and limit switch |
| Gripper Controller | ESP32 microcontroller with positional servo |
| LoRa Module (optional) | Heltec WiFi LoRa 32 V3 (915 MHz, SX1262) for remote GPS relay |
| Serial Adapter (optional) | USB-to-serial for LoRa receiver connection |
| Power | Drone battery (ANAFI Ai standard); 5 V supply for ESP32 modules |

The winch and gripper ESP32 modules connect to the drone's Wi-Fi network and communicate over WebSocket.

---

## 4. Installation and Setup

### 4.1 Clone the Repository

```bash
git clone https://github.com/GIX-Luyao/final-codebase-seahawks.git
cd final-codebase-seahawks
```

### 4.2 Install the Parrot Olympe SDK

Follow the official Parrot Olympe installation guide:

```bash
pip3 install parrot-olympe
```

Refer to the [Parrot developer documentation](https://developer.parrot.com/docs/olympe/) for platform-specific prerequisites.

### 4.3 Install Python Dependencies

Core perception dependencies:

```bash
pip3 install -r requirements.txt
```

Web GUI dependencies:

```bash
pip3 install -r gui/requirements_gui.txt
```

For optional GPU-accelerated inference:

```bash
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

### 4.4 Hardware Preparation

1. Power on the Parrot ANAFI Ai drone and wait for boot completion.
2. Connect the operator laptop to the drone's Wi-Fi network (SSID: `ANAFI Ai XXXXXX`).
3. Power on the winch and gripper ESP32 modules; verify they join the same network.
4. (Optional) Connect the LoRa receiver to the operator laptop via USB serial (`/dev/ttyUSB0`, 115200 baud).

---

## 5. How to Run the System

### 5.1 Web GUI (Recommended -- Unified Interface)

The web GUI provides integrated control for flight, perception, navigation, and winch/gripper operations.

```bash
cd gui
./start_gui.sh
```

Or directly:

```bash
python3 gui/drone_control_gui.py
```

Then open a browser at **http://localhost:5000**.

### 5.2 Demo Mode (No Hardware Required)

To explore the GUI interface without a connected drone:

```bash
python3 gui/test_gui_demo.py
```

### 5.3 Terminal-Based Manual Flight

For direct keyboard-controlled flight with live video:

```bash
python3 fly_watch_stream_ai_keyboard.py
```

Key bindings include `T` (takeoff), `L` (land), `W/A/S/D` (pitch/roll), `R/F` (altitude), `Z/E` (yaw), `I/K` (gimbal pitch), and `Q/ESC` (quit).

### 5.4 Color Tracking with Flight Control

```bash
# Calibrate HSV parameters using the drone camera (no flight commands issued)
python3 test_color_tracking.py --mode calibrate

# Full tracking: takeoff, auto-track, manual override, landing
python3 test_color_tracking.py --mode track

# Offline testing with a local webcam (no drone required)
python3 test_color_tracking.py --webcam
```

---

## 6. Expected Behavior

### Web GUI

1. The browser loads a dark-themed dashboard with tabbed panels for Navigation, Perception, and Winch System.
2. Clicking **Connect** establishes a link to the drone. Status indicators display battery level, GPS satellite count, and connection state.
3. **Takeoff** lifts the drone to approximately 1 meter hover. A virtual joystick panel becomes available for manual flight control (pitch, roll, yaw, altitude).
4. Activating Perception starts the live video feed. Detected targets are highlighted with bounding boxes overlaid on the stream.
5. In tracking mode, the drone automatically adjusts its position to center the target in the camera frame.
6. The Winch panel allows manual or automatic triggering of the lower/grip/pull retrieval sequence.
7. An emergency stop button (floating, lower-right corner) is always accessible and immediately halts all motion.

### Terminal Color Tracker

1. An OpenCV window opens displaying the annotated drone camera feed (or webcam in offline mode).
2. The HSV mask is shown in a secondary window for calibration.
3. When a qualifying target is detected, a bounding rectangle, center crosshair, and offset values are drawn on the frame.
4. In track mode, the drone issues pitch/roll/yaw corrections to minimize the target offset from frame center.

---

## 7. Key Features

- **Dual Detection Modes**: YOLOv8 deep-learning detection and HSV color-based tracking, selectable at runtime through the GUI
- **Real-Time Visual Feedback**: Live annotated video stream accessible through the web interface or OpenCV window
- **Closed-Loop Drone Alignment**: Proportional control loop that continuously adjusts drone position to center the target, with EMA smoothing and configurable dead zone
- **Automated Retrieval Workflow**: Coordinated winch lowering, gripper actuation, and cable retraction triggered upon stable target lock
- **Web-Based Control Interface**: Responsive Flask + Vue.js GUI with tabbed layout, virtual joystick, and real-time status updates
- **GPS Waypoint Navigation**: Autonomous flight to specified coordinates with configurable satellite requirements and arrival threshold
- **LoRa GPS Relay**: Receives target coordinates from a remote ground station via LoRa radio (Heltec V3, 915 MHz)
- **Safety Controls**: Emergency stop, manual override priority over autopilot, safe-mode fallback, and automatic hover on tracking loss
- **Hardware-Independent Testing**: Demo GUI mode and webcam-only color tracking allow full software verification without drone hardware

---

## 8. Technical Notes

### Architecture

The system follows a modular architecture composed of three primary subsystems, unified through the web GUI:

- **Drone Interface**: All flight commands (takeoff, landing, PCMD, moveTo) are issued through the Parrot Olympe SDK, which wraps the ANAFI Ai's internal autopilot over Wi-Fi.
- **Perception Pipeline**: Frames are captured from the drone's video stream via Olympe, decoded with OpenCV, and processed through either a YOLOv8 inference pipeline or an HSV color segmentation pipeline with morphological filtering and EMA smoothing. Detection results produce normalized offsets consumed by the tracking controller.
- **Tracking Controller**: A proportional control loop maps target pixel offsets to PCMD values for pitch, roll, yaw, and gaz. Manual keyboard inputs take priority over automatic commands with a configurable release timeout (default 0.25 s).
- **External Hardware Control**: The winch and gripper are controlled by ESP32 microcontrollers running WebSocket servers. The GUI backend communicates with them via HTTP requests (winch: `http://192.168.42.15`, gripper: `ws://192.168.42.39:81`).
- **GUI Backend**: A Flask application serves the Vue.js frontend and exposes REST endpoints for each subsystem. The backend manages drone connection lifecycle, forwards operator commands, and streams annotated video frames as base64-encoded JPEG.

### Communication Protocols

| Path | Protocol | Details |
|------|----------|---------|
| Laptop to Drone | Wi-Fi (Olympe SDK) | TCP/UDP over 192.168.42.1 |
| Laptop to Winch ESP32 | HTTP / WebSocket | `http://192.168.42.15` |
| Laptop to Gripper ESP32 | WebSocket | `ws://192.168.42.39:81` |
| LoRa TX to LoRa RX | LoRa Radio | 915 MHz, SF7, BW 125 kHz, SyncWord 0x34 |
| LoRa RX to Laptop | USB Serial | 115200 baud, `/dev/ttyUSB0` |
| GUI Frontend to Backend | HTTP + Polling | `http://localhost:5000` |

### Key Technologies

- **Parrot Olympe SDK** -- drone telemetry and flight control
- **OpenCV** -- image processing, HSV segmentation, video display
- **Ultralytics YOLOv8** -- object detection inference
- **Flask** -- web server and REST API
- **Vue.js 3** -- reactive frontend
- **Arduino / ESP32** -- embedded winch and gripper control
- **LoRa (SX1262)** -- long-range GPS coordinate relay

---

## 9. Troubleshooting

| Symptom | Likely Cause | Resolution |
|---------|-------------|------------|
| Cannot connect to drone | Laptop not on drone Wi-Fi | Connect to `ANAFI Ai XXXXXX` network; verify with `ping 192.168.42.1` |
| Takeoff fails (`drone_not_calibrated`) | Magnetometer not calibrated | Use **FreeFlight 7** app: Settings > Calibration > Magnetometer |
| Takeoff fails (other) | Low battery or propellers not installed | Charge battery; check propeller installation |
| No video in GUI or OpenCV | Olympe stream conflict or missing env var | Ensure `PYOPENGL_PLATFORM=glx` is set; close other stream consumers |
| Detection not triggering | HSV thresholds not matched to lighting | Run `test_color_tracking.py --mode calibrate` to adjust; or switch to YOLO mode in GUI |
| Winch/gripper not responding | ESP32 not on drone Wi-Fi, or wrong IP | Verify ESP32 power and Wi-Fi; confirm IP addresses in configuration |
| GPS navigation unavailable | Insufficient satellites (< 12) | Move to open outdoor area; GPS is not required for takeoff, manual control, perception, or winch |
| GUI not loading | Port 5000 occupied | Run `netstat -tlnp | grep 5000` to check; stop conflicting process |

---

## 10. Limitations

- **Lighting Sensitivity**: HSV color-based detection performance varies with ambient lighting, shadows, and water surface reflections. Threshold recalibration may be required across different environments.
- **Payload Capacity**: The Parrot ANAFI Ai has limited payload tolerance. The winch and gripper assembly must remain within the drone's weight limits.
- **GPS Dependency**: Autonomous waypoint navigation requires strong GPS signal (12+ satellites) and is not functional indoors. All other subsystems operate without GPS.
- **Single Target Tracking**: The current perception pipeline tracks one target at a time. Multi-target scenarios are not supported.
- **Wind Sensitivity**: High wind conditions affect drone stability and the accuracy of the suspended cable during winch operations.
- **Semi-Automated Workflow**: Tracking and retrieval can run automatically, but takeoff, mode selection, and landing require manual operator input.
- **Network Range**: All components (laptop, drone, ESP32 modules) must reside on the drone's Wi-Fi network, limiting operational range to its coverage area.

---

## 11. Additional Documentation

This repository includes a **PDF User Manual** (`user_manual.pdf`) that provides:

- Detailed hardware assembly and wiring instructions
- Step-by-step installation guide with screenshots
- Safety procedures and pre-flight checklists
- Operational guidance for each subsystem
- Configuration reference for all adjustable parameters

Please refer to the user manual for comprehensive setup and operational instructions.

---

**Repository**: [https://github.com/GIX-Luyao/final-codebase-seahawks](https://github.com/GIX-Luyao/final-codebase-seahawks)

**Project**: Seahawk -- University of Washington, Global Innovation Exchange

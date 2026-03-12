# Seahawk

A hybrid drone-assisted system for deploying and retrieving ocean monitoring devices in coastal field environments.

---

## 1. Project Overview

Seahawk is a capstone project developed to address the challenge of safely deploying and retrieving sensor devices in hazardous coastal areas. The system eliminates the need for researchers to physically enter dangerous water environments by leveraging an autonomous drone platform equipped with perception algorithms, a winch mechanism, and a servo-driven gripper.

**Problem Statement:**
Ocean researchers often need to place or recover monitoring instruments in locations that are difficult or dangerous to access on foot. Manual deployment exposes personnel to tidal surges, slippery terrain, and other coastal hazards.

**Target Users:**
Environmental researchers, marine scientists, and field engineers who require a remotely operated system for instrument deployment and retrieval.

**Core Capabilities:**
- Autonomous GPS-guided navigation to a target location
- Real-time visual target detection using YOLOv8 and HSV color-based tracking
- Closed-loop drone alignment to center the detected target in the camera frame
- Coordinated winch lowering and gripper actuation for payload retrieval
- Web-based operator interface for monitoring and manual intervention
- LoRa-based remote GPS coordinate relay from a ground station

---

## 2. Repository Structure

```
seahawk/
│
├── perception/
│   ├── object_detector.py            # YOLOv8 detection module
│   ├── test_color_tracking.py        # HSV color-based tracking with drone control
│   ├── fly_track_and_grab.py         # Integrated tracking + retrieval (HTTP)
│   ├── fly_track_and_grab_ws.py      # Integrated tracking + retrieval (WebSocket)
│   ├── fly_track_target.py           # Tracking-only flight script
│   ├── fly_watch_detection.py        # Detection visualization during flight
│   ├── external_systems.py           # Winch and gripper WebSocket controllers
│   ├── test_external_flow.py         # External system integration tests
│   ├── drone_detection_simple.py     # Simplified detection script
│   ├── requirements.txt              # Python dependencies (perception)
│   ├── yolov8n.pt                    # YOLOv8 nano pre-trained weights
│   ├── yolov8s.pt                    # YOLOv8 small pre-trained weights
│   │
│   ├── gui/
│   │   ├── drone_control_gui.py      # Flask web GUI (main application)
│   │   ├── test_gui_demo.py          # Demo mode (no drone hardware required)
│   │   ├── start_gui.sh              # Launch script
│   │   ├── menu.sh                   # Interactive launch menu
│   │   ├── check_dependencies.py     # Dependency verification utility
│   │   ├── requirements_gui.txt      # Python dependencies (GUI)
│   │   ├── templates/
│   │   │   └── index.html            # Vue.js frontend template
│   │   └── static/
│   │       ├── css/style.css
│   │       ├── js/app.js             # Vue.js application logic
│   │       ├── js/vue.global.js      # Vue.js runtime
│   │       └── img/                  # UI assets
│   │
│   └── esp/
│       ├── winch/
│       │   ├── winch_ws_control/
│       │   │   └── winch_ws_control.ino    # ESP32 winch firmware (WebSocket)
│       │   └── winch_http_control/
│       │       └── winch_http_control.ino  # ESP32 winch firmware (HTTP)
│       └── gripper/
│           └── gripper_control_ws/
│               └── gripper_control_ws.ino  # ESP32 gripper firmware (WebSocket)
│
├── navigation/
│   ├── fly_to_gps.py                 # GPS waypoint navigation with safety controls
│   ├── fly_to_gps_manual.py          # Manual GPS flight variant
│   └── fly_to_gps_old.py             # Legacy navigation script
│
├── debug/
│   ├── 00_check_env.py               # Environment verification
│   ├── 01_connect.py                 # Connection test
│   ├── 02_basic_state.py             # State query test
│   ├── 03_takeoff_land.py            # Takeoff/landing test
│   ├── 04_gimbal_test.py             # Gimbal control test
│   ├── 05–09_stream_*.py             # Video stream diagnostic scripts
│   └── ...                           # Additional debug utilities
│
├── README.md
└── user_manual.pdf
```

| Directory | Description |
|-----------|-------------|
| `perception/` | Computer vision modules, target tracking algorithms, external system controllers, and integration scripts |
| `perception/gui/` | Flask-based web control interface with Vue.js frontend |
| `perception/esp/` | Arduino/ESP32 firmware for winch and gripper microcontrollers |
| `navigation/` | GPS waypoint navigation scripts using the Parrot Olympe SDK |
| `debug/` | Numbered diagnostic scripts for incremental hardware verification |

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
| PyTorch | 1.8.0+ |
| Flask | 3.0.0 |
| Flask-SocketIO | 5.3.5 |
| websocket-client | (latest) |
| Web Browser | Chrome, Firefox, or Edge (for GUI access) |

Optional: NVIDIA GPU with CUDA 11.8+ for accelerated inference.

### Hardware

| Component | Description |
|-----------|-------------|
| Drone Platform | Parrot ANAFI Ai (Wi-Fi IP: 192.168.42.1) |
| Onboard Camera | ANAFI Ai integrated camera (used for perception) |
| Winch Controller | ESP32 microcontroller with servo motor, hall-effect sensor, and limit switch |
| Gripper Controller | ESP32 microcontroller with servo motor |
| LoRa Module (optional) | Heltec WiFi LoRa 32 V3 (915 MHz, SX1262) for remote GPS relay |
| Serial Adapter (optional) | USB-to-serial for LoRa receiver connection |
| Power | Drone battery (ANAFI Ai standard); 5V supply for ESP32 modules |

The winch and gripper ESP32 modules connect to the drone's Wi-Fi network and communicate via WebSocket.

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

Refer to the [Parrot developer documentation](https://developer.parrot.com/docs/olympe/) for platform-specific instructions and prerequisites.

### 4.3 Install Python Dependencies

For the perception and integration modules:

```bash
pip3 install -r perception/requirements.txt
```

For the web GUI:

```bash
pip3 install -r perception/gui/requirements_gui.txt
```

For optional GPU-accelerated inference:

```bash
pip3 install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

### 4.4 Flash ESP32 Firmware

1. Open the appropriate `.ino` file in the Arduino IDE:
   - Winch: `perception/esp/winch/winch_ws_control/winch_ws_control.ino`
   - Gripper: `perception/esp/gripper/gripper_control_ws/gripper_control_ws.ino`
2. Update the Wi-Fi SSID and password to match the drone's network.
3. Install the required Arduino libraries: `WiFi`, `WebSocketsServer`, `ESP32Servo`.
4. Flash the firmware to each ESP32 board.

### 4.5 Hardware Preparation

1. Power on the Parrot ANAFI Ai drone and wait for boot completion.
2. Connect the operator laptop to the drone's Wi-Fi network.
3. Power on the winch and gripper ESP32 modules; verify they connect to the same network.
4. (Optional) Connect the LoRa receiver to the operator laptop via USB serial.

---

## 5. How to Run the System

### 5.1 Web GUI (Recommended)

The web GUI provides unified control over all subsystems.

```bash
cd perception/gui
./start_gui.sh
```

Alternatively:

```bash
python3 perception/gui/drone_control_gui.py
```

Open a browser and navigate to `http://localhost:5000`.

### 5.2 Demo Mode (No Hardware Required)

To explore the GUI without a connected drone:

```bash
python3 perception/gui/test_gui_demo.py
```

### 5.3 Standalone Perception (Color Tracking)

```bash
# Calibrate HSV parameters using the drone camera (no flight commands)
python3 perception/test_color_tracking.py --mode calibrate

# Full tracking with flight control
python3 perception/test_color_tracking.py --mode track

# Offline testing with a local webcam
python3 perception/test_color_tracking.py --webcam
```

### 5.4 Integrated Tracking and Retrieval

```bash
# Track target and trigger winch/gripper sequence (WebSocket)
python3 perception/fly_track_and_grab_ws.py --classes person

# With custom parameters
python3 perception/fly_track_and_grab_ws.py --classes person \
    --lower-length 150 --pull-length 80 --stable-time 5
```

### 5.5 GPS Navigation

```bash
# Interactive mode (prompts for coordinates)
python3 navigation/fly_to_gps.py

# Command-line mode
python3 navigation/fly_to_gps.py --lat 47.6218 --lon -122.1769 --alt 2.0
```

---

## 6. Expected Behavior

Upon launching the web GUI and connecting to the drone:

1. **Connection**: The system establishes a connection to the drone via the Olympe SDK. Status indicators in the GUI display battery level, GPS satellite count, and connection state.
2. **Takeoff**: The operator initiates takeoff through the GUI. The drone ascends to a hover altitude of approximately 1 meter.
3. **Perception**: When the perception module is activated, the drone camera feed is processed in real time. Detected targets are highlighted with bounding boxes in the video stream displayed on the GUI.
4. **Tracking and Alignment**: In tracking mode, the system computes positional offsets between the target and the frame center, then issues pitch/roll/yaw corrections to align the drone above the target.
5. **Retrieval Sequence**: Once the target is stably centered for a configurable duration, the system automatically triggers the winch/gripper sequence: release gripper, lower cable, grip target, retract cable.
6. **Landing**: The operator commands landing through the GUI or keyboard shortcut. The drone descends and powers down motors.

The GUI also supports manual flight control (WASD-style virtual joystick), GPS navigation input, individual module testing, and an emergency stop button.

---

## 7. Key Features

- **Dual Detection Modes**: YOLOv8 deep-learning detection and HSV color-based tracking, selectable at runtime
- **Real-Time Visual Feedback**: Live annotated video stream accessible through the web interface
- **Closed-Loop Drone Alignment**: PID-style control loop that continuously adjusts drone position to center the target
- **Automated Retrieval Workflow**: Coordinated winch lowering, gripper actuation, and cable retraction triggered upon stable target lock
- **Modular External System Architecture**: Extensible `BaseExternalSystem` class supporting additional WebSocket-controlled peripherals
- **Web-Based Control Interface**: Responsive Flask + Vue.js GUI with tabbed navigation for each subsystem
- **GPS Waypoint Navigation**: Autonomous flight to specified coordinates with configurable arrival threshold and satellite requirements
- **LoRa GPS Relay**: Receives target coordinates from a remote ground station via LoRa radio
- **Safety Controls**: Emergency stop, manual override priority, safe-mode fallback, and automatic hover on tracking loss
- **Hardware-Independent Testing**: Demo mode and webcam mode allow software verification without drone hardware

---

## 8. Technical Notes

### Architecture

The system follows a modular architecture where each subsystem operates independently and communicates through well-defined interfaces:

- **Drone Interface**: All flight commands are issued through the Parrot Olympe SDK, which provides a Python API for the ANAFI Ai's internal autopilot.
- **Perception Pipeline**: Frames are captured from the drone's RTSP video stream via Olympe, decoded with OpenCV, and processed through either a YOLOv8 inference pipeline or an HSV color segmentation pipeline. Detection results produce normalized offsets used by the tracking controller.
- **Tracking Controller**: A proportional control loop maps target offsets to PCMD (Pilot Command) values for pitch, roll, yaw, and gaz. Manual keyboard inputs override automatic commands with a configurable release timeout.
- **External Systems**: The winch and gripper are controlled by ESP32 microcontrollers running WebSocket servers. The Python `external_systems.py` module provides `WinchController` and `GripperController` classes that abstract connection management and command protocols.
- **GUI Backend**: A Flask application serves the web frontend and exposes REST and WebSocket endpoints. The backend manages drone connection state, forwards commands, and streams annotated video frames as base64-encoded JPEG over Socket.IO.

### Communication Protocols

| Path | Protocol | Details |
|------|----------|---------|
| Laptop to Drone | Wi-Fi (Olympe SDK) | TCP/UDP over 192.168.42.1 |
| Laptop to Winch ESP32 | WebSocket | `ws://192.168.42.15` (port 80) |
| Laptop to Gripper ESP32 | WebSocket | `ws://192.168.42.39:81` |
| LoRa TX to LoRa RX | LoRa Radio | 915 MHz, SF7, BW 125 kHz |
| LoRa RX to Laptop | USB Serial | 115200 baud |
| GUI Frontend to Backend | HTTP + Socket.IO | `http://localhost:5000` |

### Key Technologies

- **Parrot Olympe SDK** for drone telemetry and flight control
- **Ultralytics YOLOv8** for object detection inference
- **OpenCV** for image processing and HSV color segmentation
- **Flask + Socket.IO** for the web control interface
- **Vue.js 3** for the reactive frontend
- **Arduino / ESP32** for embedded winch and gripper control
- **websocket-client** (Python) for ESP32 communication

---

## 9. Troubleshooting

### Drone Connection Failure

- Verify the laptop is connected to the drone's Wi-Fi network (`ANAFI Ai XXXXXX`).
- Confirm the drone IP is reachable: `ping 192.168.42.1`.
- Restart the drone if the connection was previously interrupted.

### Takeoff Failure (`drone_not_calibrated`)

- Use the **FreeFlight 7** mobile application to perform magnetometer calibration.
- Ensure propellers are properly installed and the battery is sufficiently charged.

### Camera Stream Not Displaying

- Confirm the Olympe SDK is correctly installed and the `PYOPENGL_PLATFORM` environment variable is set to `glx`.
- Check that no other process is consuming the video stream.

### Detection Not Working

- For YOLOv8: verify that `yolov8n.pt` or `yolov8s.pt` is present in the `perception/` directory.
- For color tracking: use calibrate mode (`--mode calibrate`) to adjust HSV thresholds for current lighting conditions.

### Winch or Gripper Not Responding

- Verify the ESP32 modules are powered and connected to the drone's Wi-Fi network.
- Confirm the WebSocket URLs in the configuration match the ESP32 assigned IPs.
- Test connectivity: attempt a WebSocket connection manually or use the GUI module test feature.

### GPS Navigation Unavailable

- GPS navigation requires a minimum of 12 visible satellites. This feature is not available indoors.
- All other subsystems (takeoff, manual control, perception, winch) operate without GPS.

---

## 10. Limitations

- **Lighting Sensitivity**: HSV color-based detection is sensitive to changes in ambient lighting, shadows, and reflections on water surfaces. Threshold recalibration may be required across different environments.
- **Payload Capacity**: The Parrot ANAFI Ai has limited payload capacity. The winch and gripper assembly must remain within the drone's weight tolerance.
- **GPS Dependency for Navigation**: Autonomous waypoint navigation requires strong GPS signal (12+ satellites) and is not functional indoors or in GPS-denied environments.
- **Single Target Tracking**: The current perception pipeline tracks a single target at a time. Multi-target scenarios are not supported.
- **Wind Sensitivity**: High wind conditions affect both drone stability and the accuracy of the suspended cable during winch operations.
- **Semi-Automated Workflow**: While the tracking and retrieval sequence is automated, takeoff, mode selection, and landing currently require manual operator input.
- **Network Topology**: All components (laptop, drone, ESP32 modules) must reside on the same Wi-Fi network hosted by the drone, which limits operational range to the drone's Wi-Fi coverage.

---

## 11. Additional Documentation

This repository includes a **PDF User Manual** (`user_manual.pdf`) that provides:

- Detailed hardware assembly and wiring instructions
- Step-by-step installation guide with screenshots
- Safety procedures and pre-flight checklists
- Operational guidance for each subsystem
- Configuration reference for all adjustable parameters

Please refer to the user manual for comprehensive setup and operational instructions beyond what is covered in this README.

---

**Repository**: [https://github.com/GIX-Luyao/final-codebase-seahawks](https://github.com/GIX-Luyao/final-codebase-seahawks)

**Project**: Seahawk -- University of Washington, Global Innovation Exchange

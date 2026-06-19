# ROS2 End Effector Control System
### Doosan H2515 — Autonomous Burr Grinding with YOLO Detection

---

## Overview

This system controls a Doosan H2515 robotic arm equipped with a custom end effector that includes:
- A YOLO-based camera for burr (çapak) detection
- A sander disk (zımpara) for grinding
- 4-channel load cells for force measurement
- CAN bus communication for servo and sander control
- A PyQt6 GUI for manual control and mission monitoring

The system supports two modes:
- **Simulation** — Ignition Gazebo with IK, simulated load cells, virtual sander
- **Real Hardware** — CAN bus serial connection to physical end effector

---

## System Requirements

| Requirement | Version |
|---|---|
| Operating System | **Ubuntu 22.04 LTS** |
| ROS2 | **Humble Hawksbill** |
| Ignition Gazebo | **Fortress** (via ros-gz) |
| Python | 3.10+ |

> This project was developed and tested on **Ubuntu 22.04 + ROS2 Humble**.
> Windows is NOT supported directly — use WSL2 with Ubuntu 22.04 or a native Linux install.

---

## Step 1 — Install ROS2 Humble

Follow the official guide: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

Quick install:
```bash
sudo apt install software-properties-common curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-desktop-full -y
```

Add to `~/.bashrc`:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 2 — Install ROS2 Extra Packages

```bash
sudo apt install -y \
  ros-humble-ros-gz \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-gz-ros2-control \
  ros-humble-cv-bridge \
  ros-humble-controller-manager \
  ros-humble-forward-command-controller \
  ros-humble-joint-state-broadcaster \
  python3-colcon-common-extensions
```

---

## Step 3 — Install Python Dependencies

```bash
pip install \
  PyQt6 \
  ultralytics \
  opencv-python \
  pyserial \
  numpy
```

> If `pip` is not found: `sudo apt install python3-pip -y`

---

## Step 4 — Extract and Build the Project

Extract the zip file:
```bash
unzip ros2_end_effector_ing.zip -d ~/
cd ~/ros2-end-effector
```

Build all packages:
```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

> First build takes 3–5 minutes. If errors occur, see the Troubleshooting section below.

Source the workspace:
```bash
source install/setup.bash
```

Add to `~/.bashrc` so you don't have to do this every time:
```bash
echo "source ~/ros2-end-effector/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Running the System

### Option A — Simulation Mode (Gazebo)

Open **Terminal 1** — start Gazebo + robot:
```bash
ros2 launch end_effector_ros2 gazebo.launch.py
```

Open **Terminal 2** — start all nodes + GUI:
```bash
ros2 launch end_effector_ros2 end_effector.launch.py simulation:=true use_gazebo:=true
```

In the GUI, make sure **"Simulation"** is selected (bottom right).

---

### Option B — Real Hardware Mode (CAN Bus)

Connect the CAN bus device to USB, then:
```bash
ros2 launch end_effector_ros2 end_effector.launch.py
```

In the GUI, select **"Real Hardware"** (bottom right).
The CAN status indicator will show `● CAN: Connected ✓` when ready.

---

## GUI Overview

```
┌─────────────────────────────────────────────────────────────────┐
│  Camera feed (left)          │  Mission Control (right)         │
│                              │  ⚠ EMERGENCY STOP               │
│                              │  START AUTONOMOUS                │
│                              │  STOP                            │
│  Log output (left bottom)    ├──────────────────────────────────│
│                              │  Guidance & Force                │
│  Load Cells / Force (left)   │  S1 Pan / S2 Tilt sliders        │
│                              │  Camera Box OPEN / CLOSE         │
│                              │  Send Servo Command              │
│                              │  Sander ON / OFF                 │
│                              │  YOLO Model selector             │
│                              │  System Status                   │
│                              │  Doosan Connection               │
│                              │  [ Simulation | Real Hardware ]  │
└─────────────────────────────────────────────────────────────────┘
```

---

## Autonomous Mission Flow

When **START AUTONOMOUS** is pressed:

1. **Camera Box OPEN** — servo extends camera out
2. **SCAN (5s)** — YOLO detects burrs in camera feed
3. **Camera Box CLOSE** — servo retracts camera
4. **Z Descent** — robot descends until load cells reach 25N contact force
5. **GRINDING** — sander disk spins, duration based on detection confidence
6. **RETRACT** — robot returns to safe height
7. **HOME** — robot returns to home position

---

## Node Architecture

| Node | Role |
|---|---|
| `gui_node` | PyQt6 GUI, publishes commands, displays status |
| `vision_node` | USB/Gazebo camera → YOLO detection → `/end_effector/detections` |
| `logic_node` | Mission orchestration, force control, guidance |
| `can_node` | CAN bus serial + Doosan DRFL interface |
| `gazebo_bridge` | Gazebo IK, simulated load cells, virtual sander (simulation only) |

### Topic Map

```
GUI ──► /end_effector/mission_start   ──► logic_node
GUI ──► /end_effector/emergency_stop  ──► logic_node, can_node, gazebo_bridge
GUI ──► /end_effector/servo_command   ──► can_node, gazebo_bridge
GUI ──► /end_effector/sander_only     ──► can_node, gazebo_bridge
GUI ──► /end_effector/set_mode        ──► all nodes

vision_node ──► /end_effector/detections       ──► logic_node
logic_node  ──► /end_effector/servo_command    ──► can_node, gazebo_bridge
logic_node  ──► /end_effector/go_home          ──► gazebo_bridge
can_node    ──► /end_effector/load_cells        ──► logic_node, GUI
can_node    ──► /end_effector/can_status        ──► GUI
```

---

## Project File Structure

```
ros2-end-effector/
├── src/
│   ├── end_effector_ros2/
│   │   ├── end_effector_ros2/
│   │   │   ├── gui_node.py          # PyQt6 GUI
│   │   │   ├── vision_node.py       # Camera + YOLO
│   │   │   ├── logic_node.py        # Mission logic + force control
│   │   │   ├── can_node.py          # CAN bus + Doosan DRFL
│   │   │   └── gazebo_bridge.py     # Gazebo simulation bridge
│   │   ├── launch/
│   │   │   ├── end_effector.launch.py   # Main launch
│   │   │   └── gazebo.launch.py         # Gazebo launch
│   │   ├── weights/
│   │   │   ├── YOLO26s.pt           # Primary YOLO model
│   │   │   ├── YOLOv11n.pt
│   │   │   └── YOLOv8.pt
│   │   └── models/
│   │       └── arac_sase/           # Vehicle chassis SDF model
│   └── doosan-robot2/               # Doosan H2515 ROS2 packages
│       ├── dsr_gazebo2/             # Gazebo robot model + controllers
│       ├── dsr_description2/        # URDF/SDF descriptions
│       └── ...
```

---

## Launch Parameters

### `end_effector.launch.py`

| Parameter | Default | Description |
|---|---|---|
| `simulation` | `false` | CAN simulation mode |
| `use_gazebo` | `false` | Start gazebo_bridge node |
| `use_gazebo_cam` | `false` | Use Gazebo camera feed |
| `can_port` | `/dev/ttyUSB0` | CAN serial port |
| `baudrate` | `2000000` | CAN baud rate |
| `model_name` | `YOLO26s.pt` | YOLO model file |
| `camera_index` | `0` | USB camera index |
| `doosan_ip` | `192.168.137.100` | Doosan controller IP |

### `gazebo.launch.py`

| Parameter | Default | Description |
|---|---|---|
| `model` | `h2515` | Doosan robot model |
| `color` | `white` | Robot color |

---

## CAN Bus Protocol

The end effector communicates over serial (USB-CAN adapter):

| Field | Value |
|---|---|
| Baud rate | 2,000,000 |
| Packet length | 10 bytes |
| Header byte | `0xAA` |
| Frame: `[0xAA, 0xC5, 0x03, 0x03, 0x00, S1, S2, SANDER, 0x00, 0x55]` | |
| S1 range | 0–180° (pan) |
| S2 range | 0–180° (tilt) |
| SANDER ON | `111` |
| SANDER OFF | `222` |

Load cell packet: header `0xAA`, bytes 4–7 = raw load cell values (subtract offset 80).

---

## Troubleshooting

### Build fails: missing packages
```bash
sudo apt install ros-humble-<missing-package>
# then rebuild:
colcon build --symlink-install
```

### GUI does not open (PyQt6 error)
```bash
pip install PyQt6
# or:
sudo apt install python3-pyqt6
```

### CAN not connecting
```bash
# Check if device is detected:
ls /dev/ttyUSB*
# Check permissions:
sudo chmod 666 /dev/ttyUSB0
# or add user to dialout group (permanent):
sudo usermod -aG dialout $USER
# then log out and back in
```

### Gazebo not starting
```bash
# Check Ignition Gazebo is installed:
ign gazebo --version
# If missing:
sudo apt install ros-humble-ros-gz
```

### YOLO model not found
Make sure `.pt` files exist in:
```
src/end_effector_ros2/weights/
```
After build, they are copied to:
```
install/end_effector_ros2/share/end_effector_ros2/weights/
```

### "colcon: command not found"
```bash
sudo apt install python3-colcon-common-extensions
```

---

## Hardware Setup (Real Hardware Mode)

1. Connect CAN-USB adapter to `/dev/ttyUSB0`
2. Grant permissions: `sudo chmod 666 /dev/ttyUSB0`
3. Connect USB camera (index 0 by default)
4. Launch: `ros2 launch end_effector_ros2 end_effector.launch.py`
5. Select **"Real Hardware"** in GUI
6. Wait for `● CAN: Connected ✓`

---

## Contact

Developer: Muhammed Emin
Email: eminn.muh.d@gmail.com

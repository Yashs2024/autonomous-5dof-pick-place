# Autonomous 5-DOF Robotic Arm: YOLOv8n & OpenCV Vision-Guided Pick & Place

[![ROS 2 Jazzy CI](https://github.com/Yashs2024/autonomous-5dof-pick-place/actions/workflows/ros2_ci.yml/badge.svg)](https://github.com/Yashs2024/autonomous-5dof-pick-place/actions/workflows/ros2_ci.yml)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS_2-Jazzy_Jalisco-22314E?logo=ros&logoColor=white)
![Gazebo Harmonic](https://img.shields.io/badge/Gazebo-Harmonic_(v8)-FF6F00?logo=gazebo&logoColor=white)
![YOLOv8](https://img.shields.io/badge/Ultralytics-YOLOv8n-00FFFF)
![MoveIt 2](https://img.shields.io/badge/MoveIt_2-Motion_Planning-00599C)
![OpenCV](https://img.shields.io/badge/OpenCV-Computer_Vision-5C3EE8?logo=opencv&logoColor=white)
![Python 3.12](https://img.shields.io/badge/Python-3.12-3776AB?logo=python&logoColor=white)
![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04_LTS-E95420?logo=ubuntu&logoColor=white)

An end-to-end autonomous robotic manipulation stack built from custom CAD assemblies in **ROS 2 Jazzy** and **Gazebo Harmonic**. This project features a full **Industrial Factory & Workbench** simulation environment, **Ultralytics YOLOv8n** real-time object detection with live FPS HUD, **OpenCV** overhead spatial localization, a custom analytical **3D 5-DOF Inverse Kinematics (IK)** solver (supporting both elevated pedestal and tabletop/ground targets), **MoveIt 2** motion planning, and a real-time **ESP32/Arduino hardware serial bridge**.

---

## Table of Contents
1. [Project Overview](#project-overview)
2. [System Architecture](#system-architecture)
3. [Key Engineering Features](#key-engineering-features)
4. [Mathematical Formulation (5-DOF Analytical IK)](#mathematical-formulation-5-dof-analytical-ik)
5. [Kinematic Chain & Joint Specifications](#kinematic-chain--joint-specifications)
6. [Repository Structure](#repository-structure)
7. [Installation & Setup](#installation--setup)
8. [Usage & Launch Modes](#usage--launch-modes)
9. [Hardware Integration (ESP32 / Arduino)](#hardware-integration-esp32--arduino)
10. [Author](#author)

---

## Project Overview

Traditional 6-DOF industrial manipulators have full orientation freedom, whereas **5-DOF robotic arms** are underactuated (lacking a 6th wrist yaw/roll axis) and require specialized position-only inverse kinematics coupled with wrist pitch compensation.

This project solves the complete autonomous pipeline inside an industrial workcell environment:
* **Industrial Factory Workcell (`factory_world.sdf`):** Custom Gazebo Harmonic warehouse environment featuring a heavy-duty robot workbench table (flush at $Z = 0.0\text{ m}$), safety bollards, floor hazard markings, storage racks, cargo crates, and PLC control cabinets.
* **Dual-Mode Perception (YOLOv8n + OpenCV):** An overhead RGB camera mounted above the workbench streams live frames over `ros_gz_bridge` to `yolov8_pick_place_node.py` (or `vision_pick_place_node.py`), displaying live inference FPS, localizing targets in 3D $(X, Y, Z)$, and classifying whether the object rests on the raised pedestal or directly on the workbench surface.
* **Planning & Control:** An analytical 5-DOF IK solver computes collision-free joint trajectories for **Hover**, **Grasp**, **Lift**, and **Place** states, driving Gazebo's `gz_ros2_control` `JointTrajectoryController` servers.
* **Sim-to-Real Telemetry:** A serial bridge node subscribes to `/joint_states` and streams calibrated PWM degree packets (`<S1,S2,S3,S4,S5,S6>`) over USB at 115200 baud to mirror simulated motions on physical hobby servos.

---

## System Architecture

```mermaid
graph TD
    subgraph Gazebo_Harmonic ["Gazebo Harmonic Factory Workcell (factory_world.sdf)"]
        CAM["Overhead RGB Sensor (640x480 @ 15Hz)"]
        WORLD["Industrial Workbench + 5-DOF Arm + Pedestal + Red Cube + Green Bin"]
        GZ_CTRL["gz_ros2_control (GazeboSimROS2ControlPlugin)"]
    end

    subgraph ROS2_Pipeline ["ROS 2 Jazzy Core & Autonomous Pipeline"]
        BRIDGE["ros_gz_bridge (/camera/image_raw & /clock)"]
        YOLO["YOLOv8n & 3D IK Node (yolov8_pick_place_node.py)"]
        VISION["OpenCV HSV & 3D IK Node (vision_pick_place_node.py)"]
        JTC_ARM["arm_group_controller (JointTrajectoryController)"]
        JTC_GRIP["gripper_group_controller (JointTrajectoryController)"]
        JSB["joint_state_broadcaster (/joint_states)"]
        SERIAL["Hardware Serial Bridge (servo_serial_bridge.py)"]
    end

    subgraph Physical_Hardware ["Physical Microcontroller Hardware"]
        MCU["ESP32 / Arduino (6x PWM Servos @ 115200 baud)"]
    end

    CAM -->|gz.msgs.Image| BRIDGE
    BRIDGE -->|sensor_msgs/Image| YOLO
    BRIDGE -.->|sensor_msgs/Image| VISION
    YOLO -->|FollowJointTrajectory| JTC_ARM
    YOLO -->|FollowJointTrajectory| JTC_GRIP
    JTC_ARM --> GZ_CTRL
    JTC_GRIP --> GZ_CTRL
    GZ_CTRL --> WORLD
    GZ_CTRL --> JSB
    JSB -->|sensor_msgs/JointState| SERIAL
    SERIAL -->|"<S1,S2,S3,S4,S5,S6>"| MCU
```

### Autonomous State Machine Workflow

```mermaid
stateDiagram-v2
    [*] --> Scanning : Launch YOLOv8n / OpenCV Vision Node
    Scanning --> TargetLocked : Object Detected + Live FPS HUD (5 Consecutive Frames)
    TargetLocked --> SurfaceCheck : Sample Border Ring Luminance
    SurfaceCheck --> SolveIK_Pedestal : Mean Intensity < 135 (Z = 0.131m)
    SurfaceCheck --> SolveIK_Ground : Mean Intensity >= 135 (Z = 0.015m)
    SolveIK_Pedestal --> ExecutePick : Compute (q1, q2, q3, q4, q5)
    SolveIK_Ground --> ExecutePick : Compute (q1, q2, q3, q4, q5)
    ExecutePick --> DropInBin : Home to Hover to Grasp to Lift to Rotate
    DropInBin --> Scanning : Release Cube & Return Home (Auto Re-Arm if Cube Moved)
```

---

## Key Engineering Features

* **Industrial Factory & Workbench Environment (`worlds/factory_world.sdf`):** Places the entire robotic workcell atop an industrial table at $Z = 0.0\text{ m}$ with warehouse floor level at $Z = -0.75\text{ m}$, preserving exact kinematic coordinates while providing realistic industrial surroundings.
* **YOLOv8n Real-Time Inference & FPS HUD (`yolov8_pick_place_node.py`):** Runs Ultralytics `yolov8n.pt` inference on the overhead camera stream with real-time frame rate (`FPS`) calculation and 3D coordinate projection.
* **CAD-to-URDF Mesh Pipeline (`arm_description`):** Leveled, centered, and scaled binary STL meshes (`0.001` mm-to-m scale) with custom inertial tensors, high-friction finger contact surfaces ($\mu_1 = \mu_2 = 50.0$, $k_p = 10^5$), viscous joint damping, and tuned `gz_ros2_control` proportional gains ($K_p = 1.8\text{--}20.0$) for stable, slip-free grasping.
* **MoveIt 2 Integration (`arm_moveit_config`):** Configured with `KDLKinematicsPlugin` (`position_only_ik: true`), OMPL `RRTConnect` planning, and `AddTimeOptimalParameterization` velocity/acceleration scaling.
* **Dual-Elevation Computer Vision (`arm_pick_place`):** Samples a 12-pixel luminance ring around the target bounding box to distinguish raised pedestal targets ($Z = 0.1309\text{ m}$) from workbench surface targets ($Z = 0.0150\text{ m}$).
* **Continuous Re-Pick Capability:** Automatically monitors the workspace after placing the object in the green collection bin; dragging the cube to any new $(X, Y, Z)$ location on the pedestal or workbench triggers a new IK calculation and pick cycle immediately.

---

## Mathematical Formulation (5-DOF Analytical IK)

### 1. Overhead Camera 2D-to-3D Projection
Given the downward-facing camera at $(X_c = 0.18\text{ m}, Y_c = 0.05\text{ m}, Z_c = 0.62\text{ m})$ with horizontal field of view $\text{HFOV} = 1.20\text{ rad}$ and resolution $640 \times 480$:

$$f_{px} = \frac{640 / 2}{\tan(1.20 / 2)} \approx 467.74\text{ px}, \qquad \Delta Z = Z_c - Z_{target}$$

$$X_{world} = X_c - \frac{(v - 240)}{f_{px}} \cdot \Delta Z, \qquad Y_{world} = Y_c - \frac{(u - 320)}{f_{px}} \cdot \Delta Z$$

### 2. Planar 2-Link Inverse Kinematics & Wrist Compensation
With link lengths $L_1 = 0.11423\text{ m}$ (`joint_2` to `joint_3`) and $L_2 = 0.12982\text{ m}$ (`joint_3` to `joint_5`), radial distance $r = \sqrt{X_{world}^2 + Y_{world}^2}$, and lateral gripper offset compensation $\Delta\psi$:

$$\theta_1 = \text{atan2}(Y_{world}, X_{world}) - \Delta\psi$$

$$\cos(\theta_3) = \frac{d^2 - L_1^2 - L_2^2}{2 L_1 L_2}, \qquad \theta_3 = \arccos(\cos\theta_3)$$

$$\theta_2 = \text{atan2}(r_w, z_w) - \text{atan2}(L_2 \sin\theta_3, L_1 + L_2 \cos\theta_3)$$

$$\theta_5 = \Phi_{desired} - (\theta_2 + \theta_3)$$

---

## Kinematic Chain & Joint Specifications

| Joint Name | Type | Parent to Child | Axis | Limits (rad) | Max Vel (rad/s) | Max Accel (rad/s^2) | Role |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| `joint_1` | Revolute | `base_link` -> `waist_link` | `[0, 0, 1]` | `[-3.14, 3.14]` | `2.0` | `2.0` | Waist Yaw (q1) |
| `joint_2` | Revolute | `waist_link` -> `arm_01_link` | `[0, 1, 0]` | `[-1.57, 1.57]` | `2.0` | `2.0` | Shoulder Pitch (q2) |
| `joint_3` | Revolute | `arm_01_link` -> `arm_02_link` | `[0, 1, 0]` | `[-2.00, 2.00]` | `2.0` | `2.0` | Elbow Pitch (q3) |
| `joint_4` | Revolute | `arm_02_link` -> `arm_03_link` | `[0.03, 0, 0.99]` | `[-3.14, 3.14]` | `2.5` | `2.5` | Forearm Roll (q4) |
| `joint_5` | Revolute | `arm_03_link` -> `gripper_base_link` | `[0, 1, 0]` | `[-1.57, 1.57]` | `3.0` | `3.0` | Wrist Pitch (q5) |
| `gripper_left_joint` | Revolute | `gripper_base_link` -> `left_finger_link` | `[0.99, 0, 0.07]` | `[-0.25, 0.60]` | `1.0` | `1.0` | Left Gripper Jaw |
| `gripper_right_joint` | Revolute | `gripper_base_link` -> `right_finger_link` | `[-0.99, 0, -0.07]` | `[-0.25, 0.60]` | `1.0` | `1.0` | Right Gripper Jaw |

---

## Repository Structure

```text
arm_ws/
└── src/
    ├── arm_description/                  # Robot URDF/Xacro, cleaned STL meshes, SDF world models
    │   ├── launch/
    │   │   └── gazebo.launch.py          # Launches factory_world.sdf, robot, cube, camera & controllers
    │   ├── meshes/                       # Binary STL meshes (base, waist, arm_01..03, gripper, gears, fingers)
    │   ├── urdf/
    │   │   ├── arm.urdf.xacro            # Parametric URDF + pedestal + green bin + gz_ros2_control
    │   │   ├── overhead_camera.sdf       # Downward-facing RGB sensor model (640x480 @ 15Hz)
    │   │   └── target_cube.sdf           # High-friction target object
    │   └── worlds/
    │       └── factory_world.sdf         # Industrial warehouse workcell & robot workbench table
    ├── arm_moveit_config/                # MoveIt 2 configuration package
    │   ├── config/
    │   │   ├── five_dof_arm.srdf         # Planning groups (arm_group, gripper_group) & collision matrix
    │   │   ├── joint_limits.yaml         # Velocity & acceleration limits for TOTG
    │   │   ├── kinematics.yaml           # KDL position_only_ik solver settings
    │   │   └── ros2_controllers.yaml     # JointTrajectoryController configurations
    │   └── launch/
    │       └── demo.launch.py            # Interactive RViz2 motion planning demo
    └── arm_pick_place/                   # Autonomous YOLOv8n/OpenCV Vision, 3D IK & Hardware Bridge
        ├── arm_pick_place/
        │   ├── yolov8_pick_place_node.py # YOLOv8n detector + live FPS HUD + 3D 5-DOF analytical IK
        │   ├── vision_pick_place_node.py # OpenCV HSV tracker + 3D 5-DOF analytical IK solver
        │   ├── pick_place_node.py        # Standalone deterministic trajectory action client node
        │   └── servo_serial_bridge.py    # ROS 2 /joint_states to USB Serial PWM packet streamer
        ├── firmware/
        │   └── esp32_arm_controller.ino  # Microcontroller firmware for 6 physical PWM servos
        └── launch/
            └── autonomous_pick_place.launch.py # Single-command master launch (Gazebo + YOLOv8n + Serial)
```

---

## Installation & Setup

### 1. System Requirements
* **OS:** Ubuntu 24.04 LTS (Noble Numbat) / WSL2 with GUI support
* **ROS 2 Distribution:** ROS 2 Jazzy Jalisco
* **Simulator:** Gazebo Harmonic (`gz-sim8`)

### 2. Install Dependencies
```bash
sudo apt update && sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-moveit \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  python3-colcon-common-extensions \
  python3-opencv \
  python3-serial \
  python3-pip

# Install Ultralytics YOLOv8 for ROS 2 Jazzy (Ubuntu 24.04 PEP 668)
pip3 install --break-system-packages ultralytics
```

### 3. Build the Workspace
```bash
git clone https://github.com/Yashs2024/autonomous-5dof-pick-place.git ~/arm_ws
cd ~/arm_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Usage & Launch Modes

### Mode 1: Full Autonomous YOLOv8n Pick-and-Place (Single Command)
Launches the Gazebo Harmonic Factory Workcell, spawns all entities on the workbench, activates `ros2_control` controllers, starts the YOLOv8n 3D IK node with live FPS overlay, and runs the hardware serial bridge:
```bash
source ~/arm_ws/install/setup.bash
ros2 launch arm_pick_place autonomous_pick_place.launch.py
```

### Mode 2: Step-by-Step Manual Launch (Two Terminals)
**Terminal 1 — Launch Gazebo Harmonic Factory Workcell:**
```bash
source ~/arm_ws/install/setup.bash
ros2 launch arm_description gazebo.launch.py
```

**Terminal 2 — Run the YOLOv8n Vision & 3D IK Node (or OpenCV HSV Node):**
```bash
source ~/arm_ws/install/setup.bash
ros2 run arm_pick_place yolov8_pick_place_node
# Or run the classic OpenCV HSV node:
# ros2 run arm_pick_place vision_pick_place_node
```

### Mode 3: MoveIt 2 Interactive RViz Planning Demo
```bash
source ~/arm_ws/install/setup.bash
ros2 launch arm_moveit_config demo.launch.py
```

---

## Hardware Integration (ESP32 / Arduino)

1. Flash `src/arm_pick_place/firmware/esp32_arm_controller.ino` to your Arduino or ESP32 using the Arduino IDE.
2. Connect your 6 servos to PWM pins `3, 5, 6, 9, 10, 11` with an external 5V-6V power supply (common ground with the microcontroller).
3. Plug the microcontroller into USB (`/dev/ttyUSB0` or `/dev/ttyACM0`) and run:
```bash
ros2 run arm_pick_place servo_serial_bridge --ros-args -p serial_port:=/dev/ttyUSB0
```

---

## Author

**Yash Surve**  
*Automation & Robotics Engineering*

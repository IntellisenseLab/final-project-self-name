# Vision-Based Leader–Follower Platooning
### CS3340: Robotics and Automation — University of Moratuwa

A ROS2-based autonomous platooning system where a Kobuki-based QBot follows a manually controlled RC car leader using AprilTag detection, Kinect v1 depth sensing, PID control, and LDROBOT D500 LiDAR for localization and navigation.

---

## Team

| Index | Name | Responsibilities |
|-------|------|-----------------|
| 230185B | Fernando S.D | ROS2 workspace, sensor interfacing, IMU & encoder testing |
| 230219K | Gunawardena H.A | AprilTag detection, depth estimation, leader tracking |
| 230626F | Suraweera B.U.D | Hardware setup, PID controller, safety monitor, lidar integration, bringup |

---

## Hardware

| Component | Details |
|-----------|---------|
| Follower Robot | Kobuki base |
| Camera | Microsoft Kinect v1 (Xbox 360) |
| LiDAR | LDROBOT D500 (LD19) |
| Leader Vehicle | Remote controlled car with AprilTag marker (tag36h11, ID 5, 12.7cm) |
| Compute | Raspberry Pi 5 (4GB) — Ubuntu 24.04 LTS |

### Physical Setup

```
[Kinect]  --USB 3.0 (blue port)--> [Pi 5]
[Kobuki]  --USB 2.0 (black port)--> [Pi 5]
[LiDAR]   --USB (via adapter)-----> [Pi 5]
[Kinect]  --power brick-----------> [power source]
[Pi 5]    --USB-C power-----------> [battery pack]
```

---

## System Overview

```
RC Car (AprilTag marker)
        │
        │  Kinect v1 detects tag
        ▼
  leader_detection node
  (TF lookup → distance & angle error)
        │
        ▼
   pid_controller node          ←── safety_monitor node
  (velocity commands)                (emergency stop /
        │                             obstacle detection /
        ▼                             return-to-home)
   kobuki_ros_node (/cmd_vel)
        │
        ▼
   Kobuki base motors

   LDROBOT D500 LiDAR
        │
        ▼
   AMCL (localization)
        │
        ▼
   Nav2 (return-to-home navigation)
```

---

## Coordinate Frame Notes

The Kinect publishes poses in the camera frame where:
- **z** — forward distance to tag
- **x** — lateral offset (left/right)
- **y** — vertical offset (up/down)

Angular error is computed as `atan2(x, z)` and negated for correct steering direction.
Distance uses full 3D Euclidean distance: `sqrt(x² + y² + z²)`.

---

## Prerequisites

- Ubuntu 24.04 LTS
- ROS2 Jazzy Jalisco
- Gazebo Harmonic (simulation only, development laptop)

### Installing Dependencies

```bash
# Kinect v1 driver dependency
sudo apt install libfreenect-dev

# ROS2 packages
sudo apt install ros-jazzy-apriltag-ros
sudo apt install ros-jazzy-image-proc
sudo apt install ros-jazzy-depth-image-proc
sudo apt install ros-jazzy-rqt-image-view
sudo apt install ros-jazzy-cv-bridge
sudo apt install ros-jazzy-image-transport
sudo apt install ros-jazzy-camera-info-manager
sudo apt install ros-jazzy-image-tools
sudo apt install ros-jazzy-nav2-bringup
sudo apt install ros-jazzy-slam-toolbox

# Python dependencies
pip install transforms3d
```

---

## Workspace Setup

```bash
# Create workspace
mkdir -p ~/ros_project_ws/src
cd ~/ros_project_ws/src

# Clone this repository
git clone <repo-url> .

# Install dependencies
cd ~/ros_project_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build

# Source (add to ~/.bashrc to avoid repeating)
source install/setup.bash
```

---

## Package Structure

```
ros_project_ws/
├── src/
│   ├── kinect_ros2/               # Kinect v1 ROS2 driver (C++)
│   │
│   ├── ldlidar_ros2/              # LDROBOT D500 LiDAR driver (C++)
│   │
│   ├── qbot_description/          # ament_cmake — Kobuki URDF, Gazebo plugins, Nav2 config
│   │   ├── urdf/
│   │   ├── sdf/
│   │   ├── config/
│   │   │   ├── nav2_params.yaml
│   │   │   ├── slam.yaml
│   │   │   └── qbot_controllers.yaml
│   │   ├── maps/
│   │   │   ├── lab_map.pgm
│   │   │   └── lab_map.yaml
│   │   └── launch/
│   │       ├── gazebo.launch.py
│   │       ├── slam.launch.py
│   │       ├── nav2.launch.py
│   │       └── navigation_launch.py
│   │
│   ├── leader_detection/          # ament_python — AprilTag detection + TF lookup
│   │   ├── leader_detection/
│   │   │   ├── __init__.py
│   │   │   └── detection_node.py
│   │   ├── config/
│   │   │   └── tags.yaml
│   │   └── launch/
│   │       └── apriltag_kinect.launch.py
│   │
│   ├── platooning_pid/            # ament_python — PID speed & steering control
│   │   └── platooning_pid/
│   │       ├── __init__.py
│   │       ├── pid_node.py
│   │       └── mock_leader.py
│   │
│   ├── safety_monitor/            # ament_python — Emergency stop, obstacle detection, return-to-home
│   │   └── safety_monitor/
│   │       ├── __init__.py
│   │       └── safety_node.py
│   │
│   ├── platooning_bringup/        # ament_cmake — System launch files
│   │   ├── config/
│   │   │   └── nav2_params.yaml
│   │   └── launch/
│   │       ├── standalone.launch.py
│   │       ├── standalone_with_nav2.launch.py
│   │       ├── standalone_with_slam.launch.py
│   │       ├── laptop.launch.py
│   │       ├── pi.launch.py
│   │       └── simulation.launch.py
│   │
│   └── ThirdParty/                # Kobuki driver dependencies
│       ├── kobuki_core/
│       ├── kobuki_ros/
│       └── ecl/
│
└── README.md
```

---

## Key ROS2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/image_raw` | `sensor_msgs/Image` | Kinect RGB feed |
| `/depth/image_raw` | `sensor_msgs/Image` | Kinect depth feed |
| `/camera_info` | `sensor_msgs/CameraInfo` | Camera calibration info |
| `/scan` | `sensor_msgs/LaserScan` | LiDAR scan data |
| `/detections` | `apriltag_msgs/AprilTagDetectionArray` | Detected AprilTags |
| `/leader/pose` | `geometry_msgs/PoseStamped` | Leader position in camera frame |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands to Kobuki |
| `/odom` | `nav_msgs/Odometry` | Wheel encoder odometry |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Robot localized position in map frame |
| `/map` | `nav_msgs/OccupancyGrid` | Occupancy map for navigation |
| `/emergency_stop` | `std_msgs/Bool` | Safety stop signal |

---

## Running the System

### Option 1: Basic platooning (no navigation)

```bash
ros2 launch platooning_bringup standalone.launch.py
```

### Option 2: Platooning with Nav2 return-to-home

```bash
ros2 launch platooning_bringup standalone_with_nav2.launch.py
```

> **Note:** AMCL requires a few seconds to localize after startup using global localization. The home position is saved automatically once AMCL publishes its first pose estimate.

### Option 3: Split setup (Pi + Laptop over network)

Set matching `ROS_DOMAIN_ID` on both machines:
```bash
export ROS_DOMAIN_ID=42
```

**On Pi:**
```bash
ros2 launch platooning_bringup pi.launch.py
```

**On Laptop:**
```bash
ros2 launch platooning_bringup laptop.launch.py
```

### Generating a new map (SLAM)

```bash
# Terminal 1 — full robot system
ros2 launch platooning_bringup standalone.launch.py

# Terminal 2 — SLAM
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/path/to/ros_project_ws/install/qbot_description/share/qbot_description/config/slam.yaml

# Terminal 3 — teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save map when done
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "name: {data: '/home/buvindu/lab_map'}"
```

### Testing PID without hardware

```bash
ros2 run platooning_pid pid_node
ros2 run platooning_pid mock_leader --ros-args -p mode:=static -p static_x:=1.5
ros2 topic echo /cmd_vel
```

---

## PID Parameters

All gains are tunable at runtime without rebuilding:

```bash
ros2 run platooning_pid pid_node --ros-args \
  -p desired_distance:=0.8 \
  -p linear_kp:=0.6 \
  -p linear_ki:=0.02 \
  -p linear_kd:=0.1 \
  -p angular_kp:=1.5 \
  -p angular_kd:=0.2 \
  -p pose_timeout:=1.5
```

---

## Safety Monitor Parameters

```bash
ros2 run safety_monitor safety_monitor --ros-args \
  -p min_following_distance:=0.3 \
  -p obstacle_distance:=0.4 \
  -p leader_timeout:=3.0 \
  -p recovery_rotation_time:=15.0 \
  -p recovery_angular_speed:=1.0
```

---

## Safety Behaviours

The system implements a four-state safety state machine:

| State | Trigger | Behaviour |
|-------|---------|-----------|
| `following` | Normal operation | PID follows leader |
| `emergency` | Leader too close / obstacle detected | Stop immediately |
| `recovering` | Leader not seen for `leader_timeout` seconds | Rotate to reacquire tag |
| `lost` | Recovery rotation timed out | Attempt Nav2 return-to-home |
| `returning` | Nav2 goal accepted | Navigate back to starting position |

### Return-to-Home (Nav2)

When the leader cannot be reacquired after the recovery rotation, the system attempts to navigate the robot back to its starting position using Nav2 and AMCL localization. The starting position is saved automatically from the first AMCL pose received after startup.

> **Note:** The return-to-home feature requires a pre-generated map of the operating environment and depends on the quality of AMCL localization. Performance is sensitive to map coverage and the match between the lidar scan and the pre-built map. Results may vary depending on environmental conditions and map quality.

---

## Development Timeline

| Week | Focus |
|------|-------|
| Week 1 | AprilTag detection, URDF model, PID controller, Gazebo simulation |
| Week 2 | Hardware setup, Ubuntu + ROS2 on Pi, Kinect driver, full pipeline test |
| Week 3 | Real robot testing with Kobuki, LiDAR integration, SLAM, Nav2 |
| Week 4 | Debugging, PID tuning, return-to-home, system integration |

---

## Known Issues and Limitations

- Kinect v1 publishes at ~5Hz on Pi under load — offloading detection to laptop improves to ~30Hz
- LiDAR occasionally drops packets during USB serial communication (harmless, SLAM continues)
- AMCL global localization requires a few seconds after startup before home position is saved
- Return-to-home navigation performance depends on map quality and AMCL convergence
- `qbot_description` Gazebo simulation requires manual lifecycle activation of Nav2 nodes

---

## Conventions

- All Python nodes use `rclpy`
- PID gains and safety thresholds are ROS2 parameters — tune via CLI without rebuilding
- `ament_cmake` for packages with no runnable Python nodes (description, bringup)
- `ament_python` for packages with runnable nodes (detection, controller, safety)
- Poses from detection are in **camera frame** — z is forward, x is lateral

---

## References

1. A. Author et al., "A Set-Theoretic Control Strategy for a Platoon of Constrained Differential-Drive Robots," *IEEE Transactions on Automation Science and Engineering*, Jan. 2026.
2. "Semi-autonomous truck platooning — how does it work?", YouTube, Feb. 2021.
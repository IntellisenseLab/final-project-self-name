# Vision-Based Leader–Follower Platooning
### CS3340: Robotics and Automation — University of Moratuwa
 
A ROS2-based autonomous platooning system where a Kobuki-based QBot follows a manually controlled RC car leader using AprilTag detection, Kinect v1 depth sensing, and PID control.
 
---
 
## Team
 
| Index | Name | Responsibilities |
|-------|------|-----------------|
| 230185B | Fernando S.D | ROS2 workspace, sensor interfacing, IMU & encoder testing |
| 230219K | Gunawardena H.A | AprilTag detection, depth estimation, leader tracking |
| 230626F | Suraweera B.U.D | Hardware setup, PID controller, URDF model, obstacle avoidance |
 
---
 
## Hardware
 
| Component | Details |
|-----------|---------|
| Follower Robot | Kobuki-based QBot |
| Camera | Microsoft Kinect v1 (Xbox 360) |
| Leader Vehicle | Remote controlled car with AprilTag marker |
| Compute | Raspberry Pi 5 |
 
---
 
## System Overview
 
```
Leader Vehicle (RC car + AprilTag)
        │
        │  Kinect v1 RGB-D Camera detects tag
        ▼
  leader_detection node
  (pose + depth → distance & angle error)
        │
        ▼
   pid_controller node
  (compute velocity commands)
        │
        ├──── safety_monitor node (obstacle / emergency stop)
        │
        ▼
   Kobuki base motors (/cmd_vel)
```
 
---
 
## Prerequisites
 
- Ubuntu 24.04 LTS
- ROS2 Jazzy Jalisco
- Gazebo Harmonic (for simulation)
- `libfreenect` + `freenect_stack` ROS2 package (Kinect v1 driver)
- `kobuki_ros` ROS2 package (Kobuki base driver)
- `apriltag_ros` package

> **Note:** Compatibility of the libraries with ROS2 Jazzy is not guaranteed yet. Needs to be checked.
--- 
### Installing Dependencies
 
```bash
# Kinect v1 driver
sudo apt install libfreenect-dev
 
# ROS2 dependencies
sudo apt install ros-jazzy-apriltag-ros
sudo apt install ros-jazzy-kobuki-ros
 
# Python dependencies
pip install transforms3d
```
 
---
 
## Workspace Setup
 
```bash
# Create workspace
mkdir -p ~/ros_project_ws//src
cd ~/ros_project_ws//src
 
# Clone this repository
git clone <repo-url> .
 
# Install dependencies
cd ~/ros_project_ws/
rosdep install --from-paths src --ignore-src -r -y
 
# Build
colcon build
 
# Source
source install/setup.bash
```
 
---
 
## Package Structure
 
```
ros_project_ws/
├── src/
│   ├── qbot_description/          # ament_cmake  — Kobuki URDF, Kinect v1 model, Gazebo plugins
│   │   ├── urdf/
│   │   ├── meshes/
│   │   └── launch/
│   │
│   ├── leader_detection/          # ament_python — AprilTag detection + depth estimation
│   │   └── leader_detection/
│   │       ├── __init__.py
│   │       └── detection_node.py
│   │
│   ├── pid_controller/            # ament_python — PID speed & steering control
│   │   └── pid_controller/
│   │       ├── __init__.py
│   │       └── pid_node.py
│   │
│   ├── safety_monitor/            # ament_python — Emergency stop & obstacle detection
│   │   └── safety_monitor/
│   │       ├── __init__.py
│   │       └── safety_node.py
│   │
│   └── platooning_bringup/        # ament_cmake  — Launch files that start the whole system
│       └── launch/
│           ├── simulation.launch.py
│           └── real_robot.launch.py
│
└── README.md
```
 
> **Note:** If custom messages are needed, add a `platooning_msgs` package (`ament_cmake`) inside `src/`.
 
---
 
## Key ROS2 Topics
 
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | Kinect RGB feed |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | Kinect depth feed |
| `/tag_detections` | `apriltag_msgs/AprilTagDetectionArray` | Detected AprilTags |
| `/leader/pose` | `geometry_msgs/PoseStamped` | Estimated leader position relative to robot |
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands to Kobuki base |
| `/odom` | `nav_msgs/Odometry` | Wheel encoder odometry |
| `/emergency_stop` | `std_msgs/Bool` | Safety stop signal |
 
---
 
## Running the System
 
### Simulation (Gazebo Harmonic)
 
```bash
source install/setup.bash
ros2 launch platooning_bringup simulation.launch.py
```
 
### Real Robot
 
```bash
source install/setup.bash
ros2 launch platooning_bringup real_robot.launch.py
```
 
---
 
## Development Order
 
Build the packages in this order — each step depends on the previous:
 
1. **`qbot_description`** — Robot model in Gazebo. Everything else depends on this.
2. **`pid_controller`** — Core control logic. Testable with a fake leader pose publisher.
3. **`leader_detection`** — AprilTag pipeline. Connects to the PID controller.
4. **`safety_monitor`** — Emergency stop and obstacle avoidance.
5. **`platooning_bringup`** — Launch files. Written last once all nodes are working.
 
---
 
## Development Timeline
 
| Week | Focus |
|------|-------|
| Week 1 | AprilTag detection, URDF model, PID controller, Gazebo simulation |
| Week 2 | Hardware setup, Ubuntu + ROS2 install on Pi, sensor verification|
| Week 3 | Real robot testing, SLAM, full system integration |
| Week 4 | Debugging, fail-safe behaviours, rosbag logging, RViz visualization |
 
---
 
## Safety Behaviours (Optional due to time constraints)
 
- **Emergency Stop** — Kobuki halts if distance to leader falls below minimum threshold
- **Obstacle Avoidance** — Depth camera detects obstacles independent of leader tracking
- **Leader Loss Recovery** — If AprilTag is lost, robot rotates in place to reacquire
 
---
 
## Conventions
 
- All Python nodes use `rclpy`
- PID gains and key parameters are ROS2 parameters — tune via YAML or command line without recompiling
- `ament_cmake` for packages with no runnable Python nodes (description, bringup)
- `ament_python` for packages with runnable nodes (detection, controller, safety)
 
---
 
## References
 
1. A. Author et al., "A Set-Theoretic Control Strategy for a Platoon of Constrained Differential-Drive Robots," *IEEE Transactions on Automation Science and Engineering*, Jan. 2026.
2. "Semi-autonomous truck platooning — how does it work?", YouTube, Feb. 2021.
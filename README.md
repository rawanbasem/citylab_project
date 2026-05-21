
---

# Robot Patrol & Autonomous Navigation Package (`robot_patrol`)

An autonomous robotic navigation and patrolling system developed in **ROS2 Humble (C++)**. This project demonstrates the design of custom ROS2 interfaces, synchronous obstacle avoidance services, and a custom, preemptive asynchronous Action Server for precise 2D pose navigation. The software architecture was validated in a Gazebo simulation and successfully deployed onto a physical **FastBot** differential-drive mobile robot.

## 🛠️ Tech Stack & Key Concepts

* **Framework:** ROS2 Humble Utilities (`rclcpp`, `rclcpp_action`)
* **Language:** C++ (Object-Oriented Programming, Multi-threading)
* **Simulation:** Gazebo, RViz2
* **Hardware Target:** FastBot Differential Drive Platform
* **Robotics Math:** Quaternion-to-Euler Transformations, Proportional ($P$) Control Loops, Geometric Vector Calculus

---

## 🏗️ System Architecture

The package comprises two distinct operational modes designed to show command over key core concepts of distributed robotic software engineering:

```
                  ┌──────────────────────┐
                  │  /direction_service  │ <── Custom Service
                  └──────────────────────┘
                             ▲
                             │ (Laser Scan Data / Best Direction)
                             ▼
┌──────────────┐  Publish    ┌──────────────────────┐
│  /laser_scan ├────────────>│ /patrol_with_service │
└──────────────┘             └──────────┬───────────┘
                                        │
                                        │ Publish Velocity
                                        ▼
                                 ┌──────────────┐
                                 │   /cmd_vel   │
                                 └──────────────┘

```

### 1. Synchronous Obstacle Avoidance (Task 1: Services)

* **`/direction_service` (Server):** Directs the robot out of dead ends. It segments a $180^\circ$ LiDAR laser scan array into three distinct $60^\circ$ zones (Right, Front, Left), computes total clearance metrics dynamically, and yields the safest geometric heading path (`forward`, `left`, `right`).
* **`/patrol_with_service_node` (Client):** Drives linearly at a steady cruise until an obstacle breaks a defined front range threshold limit, triggers a synchronous service request to find safety, and handles the output state smoothly.

### 2. Preemptive 2D Point-Stabilization (Task 2: Actions)

* **`/go_to_pose` (Action Server):** An asynchronous tracking system designed to pilot a mobile robot to exact spatial coordinate goals $(x, y, \theta)$.
* **Quaternions to Euler Angles:** Subscribes to `/odom` data and maps 3D orientation math down to a 2D planar heading (Yaw).
* **Proportional Control ($P$-Controller):** Implements dynamic angular velocity adjustments relative to spatial vector orientation errors to smooth out tracking profiles.
* **Multi-Goal Preemption/Management:** Uses thread-safe checks (`is_active()`) allowing any new incoming command to safely abort active operational threads, safely resetting velocities without crashing the node.
* **Throttled Asynchronous Feedback:** Broadcasts current odometry statistics tightly capped at exactly $1\text{Hz}$ back to the calling client for cleaner terminal operations.



---

## 📊 Custom Interfaces

### `GetDirection.srv`

```protobuf
# Request
sensor_msgs/LaserScan laser_data
---
# Response
string direction

```

### `GoToPose.action`

```protobuf
# Goal
geometry_msgs/Pose2D goal_pos
---
# Result
bool status
---
# Feedback
geometry_msgs/Pose2D current_pos

```

---

## 🚀 Installation & Build Guide

### Prerequisites

Ensure you have a sourcing installation of ROS2 Humble setup alongside appropriate build tools (`colcon`):

```bash
cd ~/ros2_ws/src
git clone https://github.com/<your-username>/citylab_project.git
cd ~/ros2_ws

```

### Compilation

Compile the project safely using `colcon` tracking:

```bash
colcon build --packages-select robot_patrol
source install/setup.bash

```

---

## 💻 How to Run & Verify

### Running Autonomous Patrol (Task 1)

To initialize the full obstacle avoidance tracking system along with its corresponding customized visual configurations in RViz2:

```bash
ros2 launch robot_patrol main.launch.py

```

### Running Point Stabilization Action (Task 2)

1. Launch the dedicated action server thread:
```bash
ros2 launch robot_patrol start_gotopose_action.launch.py

```


2. Open an alternative bash prompt, source your workspace, and inject coordinate spatial targets into the server pipeline:
```bash
ros2 action send_goal -f /go_to_pose robot_patrol/action/GoToPose "{goal_pos: {x: 0.70, y: 0.36, theta: 0.0}}"

```



---

## 🦾 Real-Robot Deployment Insights (The "Reality Gap")

Deploying the simulation profile over to the hardware branch hosted on physical **FastBot** units highlighted typical physical edge cases:

* **Hardware Calibration Filters:** Accommodated internal hardware sensor anomalies caused by reflection beams bouncing back off internal chassis mounting wire posts at $8\text{cm}$ ranges. Added localized noise range thresholds to mask static chassis blocks.
* **Quality of Service (QoS) Management:** Adjusted default networking connectivity parameters inside RViz visualization modules from `Reliable` to `Best Effort` state parameters to resolve packet-drop streaming blocks common across typical high-frequency remote LiDAR streams.

---



```

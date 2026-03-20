# 🚁 Autonomous GPS-Denied Drone Navigation & SLAM (Webots + ROS2)

### *Real-World Ready Autonomy for the NVIDIA Jetson Platform*

---

## 📄 Project Overview
This project proposes a **low-cost, autonomous drone** integrating **RGB-D vision**, **2D LIDAR**, and **onboard computation (NVIDIA Jetson)** to perform real-time **SLAM**, **obstacle avoidance**, and **path planning** without GPS. 

It tackles strict hackathon problem statements by utilizing the industry-standard **Webots Robotics Simulator** and **ROS2 (Robot Operating System)**.

---

## 🎯 Solved Hackathon Problem Statements

### 1-a) Simulate autonomous navigation of a drone from a start location to a goal location in a 2D environment...
**Solution:** The drone utilizes a 2D LiDAR constraint model and implements mathematical **A* Pathfinding** (`astar_navigator.py`). The logic drives the drone to waypoints while successfully dodging randomized walls generated in `worlds/obstacle_course.wbt`.

### 1-b) Simulate a drone using PID controller such that it maintains a certain vertical height and avoids obstacles.
**Solution:** The `pid_altitude_controller.py` directly manipulates 4 motor velocities based on simulated Inertial/GPS units. A strict mathematical Proportional-Integral-Derivative loop maintains steady altitude while using the front-facing LiDAR cone to forcefully pitch backward (`-2.0` velocity vector) to dodge obstacles.

### 1-c) Develop an autonomous navigation system for underground or tunnel environments where GPS signals cannot reach...
**Solution:** We built an actual 3D underground tunnel environment (`worlds/underground_tunnel.wbt`). The `Mavic 2 PRO` quadrotor is outfitted entirely with alternative sensing: **RGB-D Vision Cameras** and a high-resolution **360 LiDAR Node** enabling full VSLAM capabilities without GPS dependency. 

---

## 🏗️ Architecture Design

We migrated from lightweight web-simulators directly into **Webots**, guaranteeing physics-accurate drone dynamics and immediate ROS2 bridging:

- `worlds/`: Contains `.wbt` files describing the tunnels, physics rules, and drones.
- `controllers/`: Contains the autonomous brain logic accessing the Webots `Robot` API.
- `ros2_ws/`: Our dedicated hackathon deliverable for `webots_ros2`. It launches the Webots world while subscribing/publishing ROS2 standard standard topics (e.g. `/scan`).

---

## 🏁 Quick Start: Running the Simulation

### Option 1: Native Webots
1. Download and install **[Webots R2023b](https://cyberbotics.com/)** on your machine.
2. Open Webots.
3. Go to `File > Open World` and select `worlds/underground_tunnel.wbt` or `worlds/obstacle_course.wbt`.
4. Press the "Play" triangle at the top to watch the Python Autonomy Controllers immediately launch the drone.

### Option 2: ROS2 Launch (Colcon Setup)
If running on an Ubuntu or Docker ROS2 environment (`Humble` recommended):
```bash
# Navigate to the workspace
cd ros2_ws

# Build the autonav packages
colcon build

# Source environment
source install/setup.bash

# Launch Webots + ROS2 Driver
ros2 launch autonav_ros2 drone_launch.py
```

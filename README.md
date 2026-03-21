# 🚁 AutoNAV-AI: Autonomous Drone Navigation & SLAM

`NVIDIA Jetson` `Webots` `SLAM` `Autonomous Navigation` `A* Pathfinding` `PID Control`

---

## 📄 Project Overview
AutoNAV-AI is a high-performance autonomous drone navigation system designed for GPS-denied environments. It integrates RGB-D vision, 2D LiDAR, and onboard computation (NVIDIA Jetson) to perform real-time SLAM, obstacle avoidance, and path planning. The system utilizes the **Webots Robotics Simulator** for accurate physics simulation and modular control.

### Key Capabilities
- **GPS-Denied Navigation**: Operates in tunnels, underground, or indoor environments without relying on satellite signals.
- **A* Pathfinding**: Intelligent path planning around obstacles in dynamic environments.
- **PID Altitude Control**: Precise vertical stabilization and obstacle-aware pitching.
- **Real-time SLAM**: Simultaneous Localization and Mapping using LiDAR and Vision.
- **Web Dashboard**: An interactive interface to monitor and control simulation missions.

---

## 🏗️ Architecture Design

- `worlds/`: Webots world files (`.wbt`) defining the drone's environment and physics.
- `controllers/`: Core autonomous logic for drone behavior and sensor processing.
- `server.py`: Flask-based dashboard server.

---

## 🏁 How to Use

### 1. Requirements
Ensure you have the following installed:
- **Python 3.10+**
- **Webots R2023b**
- **Dependencies**: `pip install flask numpy matplotlib`

### 2. Start the Dashboard
Run the dashboard server to monitor the drone's status:
```bash
python server.py
```
Access the interface at [http://localhost:5000](http://localhost:5000).

### 3. Native Webots Simulation
1. Open **Webots**.
2. Go to `File > Open World` and select a world file:
   - `worlds/underground_tunnel.wbt`: For GPS-denied SLAM testing.
   - `worlds/obstacle_course.wbt`: For A* navigation testing.
3. Press the **Play** button to start the simulation and autonomy controllers.

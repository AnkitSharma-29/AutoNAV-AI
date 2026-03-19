# 🚁 Autonomous GPS-Denied Drone Navigation & SLAM

### *Bridging the gap between laboratory research and real-world deployment.*

---

## 📄 Project Overview
This project proposes a **low-cost, autonomous drone** integrating **RGB-D vision**, **2D LIDAR**, and **onboard computation (NVIDIA Jetson)** to perform real-time **SLAM**, **obstacle avoidance**, and **path planning** without GPS. 

It offers an open, reproducible platform validated through high-fidelity simulation and field testing. This strengthens strategic capabilities in autonomous navigation for critical, GPS-denied environments like underground tunnels, dense forests, and indoor warehouses.

---

## 🚀 Key Features

### 1. **Interactive 3D Simulation Dashboard**
A premium web-based visualization tool (built with **Three.js** and **Flask**) that allows real-time monitoring and stress-testing of drone algorithms.
- **Real-Time Robustness Testing:** Sliders to adjust wind turbulence, sensor noise, and obstacle density on the fly.
- **Dynamic 3D Environment:** Live rendering of drone trajectories, obstacle fields, and tunnel structures.

### 2. **Core Navigation Modules**
- **1-a) A* Path Planning:** Optimal obstacle avoidance from start to goal in 2D/3D grids.
- **1-b) PID Altitude Control:** High-precision vertical stabilization using proportional-integral-derivative logic with wind disturbance rejection.
- **1-c) GPS-Denied SLAM:** Sensor fusion using LIDAR data for lateral correction and localization in unknown 3D tunnels.

### 3. **Architectural Design Choices (Why Custom?)**
Instead of relying on heavy physics engines like Microsoft AirSim or NVIDIA Isaac Sim, this project implements a **zero-dependency, web-native kinematics engine**.
- **Algorithmic Isolation:** By abstracting raw LIDAR ray-tracing into direct 2D/3D Occupancy Grids, we isolate and prove the core A* and PID math without external engine noise.
- **Accessibility:** A decoupled Python Backend + Three.js Frontend means the entire autonomy visualization runs instantly in a browser without gigabytes of gaming-engine overhead.

---

## 🛠 Tech Stack
- **Onboard Compute:** NVIDIA Jetson (Simulated performance parameters)
- **Sensing:** RGB-D Depth Vision & 2D LIDAR
- **Software:** Python 3, Flask, Three.js, NumPy, Matplotlib
- **Framework:** ROS2 compatible architecture

---

## 🏁 Quick Start: Running the Simulation

### 1. Install Dependencies
```bash
pip install flask numpy matplotlib
```

### 2. Launch the Immersive Dashboard
```bash
python server.py
```
Then visit **[http://127.0.0.1:5000](http://127.0.0.1:5000)** in your browser.

### 3. Run Individual Modules (CLI)
- **Path Planning:** `python astar_navigation.py --density 0.2`
- **PID Control:** `python pid_altitude_controller.py --altitude 15.0 --wind 2.0`
- **SLAM Tunnel:** `python gps_denied_slam.py --noise 0.08 --drift 0.05`

---

## 🏆 Hackathon Deliverables
- [x] **Working Prototype:** Fully functional 3D simulation backend and frontend.
- [x] **Validated Algorithms:** PID, A*, and SLAM tested under variable noise/drift conditions.
- [x] **Documentation:** Comprehensive README and technical walkthrough.
- [/] **Field Ready:** ROS2-ready logic structure for deployment on NVIDIA Jetson.

---
*Developed for the Autonomous Drone AI Challenge 2026.* 🇮🇳

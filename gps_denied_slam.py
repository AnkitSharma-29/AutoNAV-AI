import numpy as np
import matplotlib.pyplot as plt
import json
import argparse
import random

class GPSDeniedDrone:
    def __init__(self, start_pos=(0, 2)):
        self.true_pos = np.array(list(start_pos), dtype=float)  # Actual position (unknown to drone)
        self.est_pos = np.array(list(start_pos), dtype=float)   # Drone's estimated position
        self.drift_error = np.zeros(2)
        
    def move(self, velocity, dt, noise_sigma=0.05):
        # Actual movement with noise
        actual_vel = velocity + np.random.normal(0, noise_sigma, 2)
        self.true_pos += actual_vel * dt
        
        # Estimated movement (Odometry only - with drift)
        self.est_pos += velocity * dt
        
    def get_lidar_readings(self, tunnel_width=4.0):
        # Simple lidar measuring distance to left and right walls of a y-aligned tunnel
        dist_left = self.true_pos[1]  # distance to y=0
        dist_right = tunnel_width - self.true_pos[1] # distance to y=4
        return dist_left, dist_right

    def sensor_fusion_update(self, dist_left, dist_right, tunnel_width=4.0):
        # Simple SLAM-like correction: use wall measurements to correct the Y-estimate
        corrected_y = (dist_left + (tunnel_width - dist_right)) / 2
        self.est_pos[1] = corrected_y

def run_slam_simulation():
    parser = argparse.ArgumentParser()
    parser.add_argument("--noise", type=float, default=0.08)
    parser.add_argument("--drift", type=float, default=0.04)
    args = parser.parse_args()

    drone = GPSDeniedDrone()
    dt = 0.1
    duration = 10.0
    steps = int(duration / dt)
    
    true_path = []
    est_path = []
    
    tunnel_width = 4.0
    target_velocity = np.array([1.0, 0.0])
    
    print(f"Starting SLAM Simulation (Noise: {args.noise}, Drift: {args.drift})")
    
    for i in range(steps):
        # 1. Prediction with noise
        drone.move(target_velocity, dt, noise_sigma=args.noise)
        
        # 2. Continuous drift
        drone.true_pos[1] += (random.random() - 0.5) * args.drift
        
        # 3. Measurement (Lidar SLAM)
        if i % 3 == 0: 
            d_left, d_right = drone.get_lidar_readings(tunnel_width)
            drone.sensor_fusion_update(d_left, d_right, tunnel_width)
        
        true_path.append(drone.true_pos.copy())
        est_path.append(drone.est_pos.copy())
    
    true_path = np.array(true_path)
    est_path = np.array(est_path)
    
    # Export JSON
    traj_data = [{"x": float(p[0]), "y": 1.0, "z": float(p[1])} for p in true_path]
    with open('slam_data.json', 'w') as f:
        json.dump(traj_data, f)

    # Plotting
    plt.figure(figsize=(10, 5))
    plt.axhline(y=0, color='black', linewidth=3, label='Tunnel Wall')
    plt.axhline(y=tunnel_width, color='black', linewidth=3)
    plt.plot(true_path[:, 0], true_path[:, 1], 'g-', label='Actual Path')
    plt.plot(est_path[:, 0], est_path[:, 1], 'r--', label='SLAM Estimate')
    plt.title(f'GPS-Denied Navigation (Noise: {args.noise})')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.savefig('gps_denied_slam_plot.png')

if __name__ == "__main__":
    run_slam_simulation()

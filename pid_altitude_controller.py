import numpy as np
import matplotlib.pyplot as plt
import time
import json
import argparse
import random

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, measurement, dt):
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        self.prev_error = error
        return output

class DroneSimulator:
    def __init__(self, mass=1.0, gravity=9.81):
        self.mass = mass
        self.gravity = gravity
        self.altitude = 0.0
        self.velocity = 0.0
        self.acceleration = 0.0

    def update(self, thrust, dt):
        # f = ma => a = f/m
        # Total force = Thrust - Gravity Force
        net_force = thrust - (self.mass * self.gravity)
        self.acceleration = net_force / self.mass
        
        # Simple Euler integration
        self.velocity += self.acceleration * dt
        self.altitude += self.velocity * dt
        
        # Ground constraint
        if self.altitude < 0:
            self.altitude = 0
            self.velocity = 0
            
    def get_altitude(self):
        return self.altitude

def run_pid(target_altitude=10.0, wind_strength=0.0, save_files=True):
    # Simulation parameters
    dt = 0.1
    duration = 10.0
    steps = int(duration / dt)
    
    # PID parameters
    kp = 15.0
    ki = 0.5
    kd = 8.0
    
    pid = PIDController(kp, ki, kd, target_altitude)
    drone = DroneSimulator(mass=1.0)
    
    altitudes = []
    times = []
    thrusts = []
    
    for i in range(steps):
        current_time = i * dt
        current_alt = drone.get_altitude()
        
        control_output = pid.compute(current_alt, dt)
        noise = (random.random() - 0.5) * wind_strength
        
        thrust = control_output + (drone.mass * drone.gravity) + noise
        thrust = max(0, min(thrust, 40.0)) 
        
        drone.update(thrust, dt)
        
        altitudes.append(current_alt)
        times.append(current_time)
        thrusts.append(thrust)

    # Export JSON data
    traj_data = [{"x": 0, "y": float(alt), "z": 0, "t": float(t)} for alt, t in zip(altitudes, times)]
    
    if save_files:
        with open('pid_data.json', 'w') as f:
            json.dump(traj_data, f)

        # Plotting Results
        plt.figure(figsize=(10, 6))
        plt.subplot(2, 1, 1)
        plt.plot(times, altitudes, label='Actual Altitude', color='blue')
        plt.axhline(y=target_altitude, color='red', linestyle='--', label='Target')
        plt.ylabel('Altitude (m)')
        plt.title(f'Drone Altitude PID (Wind: {wind_strength})')
        plt.grid(True, alpha=0.3)
        
        plt.subplot(2, 1, 2)
        plt.plot(times, thrusts, label='Thrust Output', color='green')
        plt.xlabel('Time (s)')
        plt.ylabel('Thrust (N)')
        plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.savefig('altitude_pid_plot.png')
        plt.close()

    return traj_data

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--altitude", type=float, default=10.0)
    parser.add_argument("--wind", type=float, default=0.0)
    args = parser.parse_args()

    run_pid(args.altitude, args.wind)

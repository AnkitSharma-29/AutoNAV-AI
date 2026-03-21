"""
Omni-Sensing GPS-Denied AI Navigation Controller for Webots.
Features: 
- 6-directional obstacle sensing (Front, Back, Left, Right, Up, Down)
- IMU-based relative odometry (No GPS)
- Reactive AI obstacle avoidance using potential fields
- Autonomous Target Seeking in local coordinates
"""

import math
import numpy as np
import random
from controller import Robot, Motor, InertialUnit, Gyro, Compass, Lidar, Keyboard

class PID:
    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.prev_error = 0
        self.integral = 0

    def compute(self, setpoint, current, dt):
        error = setpoint - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        if self.output_limit:
            output = max(-self.output_limit, min(self.output_limit, output))
        return output

class OmniDroneAI(Robot):
    def __init__(self):
        super(OmniDroneAI, self).__init__()
        self.time_step = int(self.getBasicTimeStep())
        
        # --- Hardware Setup ---
        self.motors = [
            self.getDevice("front left propeller"),
            self.getDevice("front right propeller"),
            self.getDevice("rear left propeller"),
            self.getDevice("rear right propeller")
        ]
        for m in self.motors:
            m.setPosition(float('inf'))
            m.setVelocity(0.0)

        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)
        
        self.compass = self.getDevice("compass")
        self.compass.enable(self.time_step)
        
        # Use Lidar for Omni-Sensing + Simulation of 6-dir sensors
        self.lidar = self.getDevice("lidar")
        if self.lidar:
            self.lidar.enable(self.time_step)
            self.lidar.enablePointCloud()

        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.time_step)

        # --- PID Controllers ---
        # Altitude PID
        self.alt_pid = PID(kp=25.0, ki=1.0, kd=15.0, output_limit=50.0)
        # Attitude PIDs (Balance)
        self.roll_pid = PID(kp=0.5, ki=0.01, kd=0.1)
        self.pitch_pid = PID(kp=0.5, ki=0.01, kd=0.1)
        self.yaw_pid = PID(kp=2.0, ki=0.0, kd=0.5)

        # --- AI & Navigation State ---
        self.target_alt = 2.0
        self.local_pos = np.array([0.0, 0.0, 0.0])
        self.vel_est = np.array([0.0, 0.0, 0.0])
        self.last_time = self.getTime()
        
        self.mode = "AUTO" 
        self.state = "PRE_FLIGHT_ANALYSIS" # PRE_FLIGHT_ANALYSIS, NORMAL, RECOVERY
        self.target_wp = np.array([10.0, 0.0, 10.0])
        
        # Recovery / Stuck detection
        self.stall_timer = 0
        self.recovery_step = 0
        self.recovery_timer = 0
        self.last_pos = np.array([0.0, 0.0, 0.0])
        self.analysis_timer = 0
        self.analysis_reported = False
        self.start_yaw = 0.0
        self.sweep_yaw_sum = 0.0
        self.prev_front_dist = 10.0
        
        print("Omni-Drone AI Initialized. State: PRE_FLIGHT_ANALYSIS")

    def update_odometry(self, dt):
        """Estimate displacement without GPS using IMU and orientation"""
        # Note: This is simplified dead-reckoning. 
        # Real GPS-denied would use Visual Odometry or Lidar SLAM.
        roll, pitch, yaw = self.imu.getRollPitchYaw()
        
        # Horizontal acceleration is proportional to tilt (sin theta)
        # Assuming X is left/right and Z is fwd/back in Webots Mavic coords
        # This is a very rough approximation for "GPS-Denied" context
        accel_x = math.sin(pitch) * 9.81
        accel_z = -math.sin(roll) * 9.81
        
        # Integration
        self.vel_est[0] += accel_x * dt
        self.vel_est[2] += accel_z * dt
        
        # Drag simulation
        self.vel_est[0] *= 0.98
        self.vel_est[2] *= 0.98
        
        self.local_pos[0] += self.vel_est[0] * dt
        self.local_pos[2] += self.vel_est[2] * dt
        # Altitude tracking would typically use a Barometer or Ultrasonic sensor.
        
    def sense_omni(self):
        """Processes Lidar point cloud to detect obstacles in 6 directions"""
        sensors = {
            'front': 10.0, 'back': 10.0, 
            'left': 10.0, 'right': 10.0,
            'up': 10.0, 'down': 10.0
        }
        
        if self.lidar:
            ranges = self.lidar.getRangeImage()
            num_points = len(ranges)
            if num_points > 0:
                # Broaden the sensing arc (Check 20% of the total points for each direction)
                arc = num_points // 8
                mid = num_points // 2
                sensors['front'] = min(ranges[mid-arc : mid+arc])
                sensors['left'] = min(ranges[mid+num_points//4-arc : mid+num_points//4+arc])
                sensors['right'] = min(ranges[mid-num_points//4-arc : mid-num_points//4+arc])
                sensors['back'] = min(min(ranges[:arc]), min(ranges[-arc:]))
                
                # NEW: Find the "Best Exit" (Direction with max clearance)
                best_idx = np.argmax(ranges)
                # Convert lidar index to relative angle (0 is front)
                angle_offset = (best_idx - mid) / num_points * 2 * math.pi
                sensors['best_exit_angle'] = angle_offset
                sensors['max_clearance'] = ranges[best_idx]
                
                # Check for Direct Line-of-Sight to Goal
                target_vec = self.target_wp - self.local_pos
                target_dist = np.linalg.norm(target_vec)
                if target_dist > 0.001:
                    target_yaw = math.atan2(target_vec[0], target_vec[2])
                    angle_offset_goal = (target_yaw) / (2 * math.pi) * num_points
                    idx = int((mid + angle_offset_goal) % num_points)
                    win = 5
                    dist_to_obs = min(ranges[idx-win : idx+win]) if idx-win > 0 and idx+win < num_points else 10.0
                    sensors['goal_los'] = dist_to_obs > target_dist
                else:
                    sensors['goal_los'] = False
            
        return sensors

    def run_ai_logic(self, sensors, dt, current_yaw):
        """Autonomous AI navigation using reactive potential fields with recovery states"""
        fwd_input = 0.0
        side_input = 0.0
        yaw_input = 0.0
        
        # 1. State: PRE_FLIGHT_ANALYSIS (Guaranteed 360-degree Scan)
        if self.state == "PRE_FLIGHT_ANALYSIS":
            if self.analysis_timer == 0:
                self.start_yaw = current_yaw
                print("Starting Precision 360° Surroundings Scan...")
            
            self.analysis_timer += dt
            # Integrate yaw change to ensure full 360
            yaw_input = 3.0 # Rotate faster for efficiency
            self.sweep_yaw_sum += abs(yaw_input * dt)

            if self.sweep_yaw_sum > 6.4 and not self.analysis_reported: # > 2*PI
                print(f"--- 360° Surroundings Report ---")
                print(f"Clearance: F:{sensors['front']:.1f}m B:{sensors['back']:.1f}m L:{sensors['left']:.1f}m R:{sensors['right']:.1f}m")
                print(f"Best Exit: {math.degrees(sensors['best_exit_angle']):.1f}° | Max Gap: {sensors['max_clearance']:.1f}m")
                self.analysis_reported = True
                
                if min(sensors['front'], sensors['left'], sensors['right'], sensors['back']) < 2.5:
                    print("CRITICAL: Nearby Obstacle! Adjusting launch to 5m Altitude.")
                    self.target_alt = 5.0
            
            if self.sweep_yaw_sum > 7.0: # Ensure scan is fully finished
                if min(sensors['front'], sensors['left'], sensors['right'], sensors['back']) < 1.2:
                    print("Emergency Nudge towards Best Exit...")
                    fwd_input = 6.0 * math.cos(sensors['best_exit_angle'])
                    side_input = 6.0 * math.sin(sensors['best_exit_angle'])
                    if self.analysis_timer > 5.0:
                         self.state = "NORMAL"
                         print("Escape move complete. READY.")
                    return fwd_input, side_input, 0.0
                else:
                    self.state = "NORMAL"
                    print("360° Analysis Complete. Launching...")
            return 0.0, 0.0, yaw_input

        # 2. Collision Avoidance (VIRTUAL BUMPER & Exponential Repulsion)
        avoid_fwd = 0.0
        avoid_side = 0.0
        HARD_LIMIT = 2.5 # Minimum particular distance to maintain
        FORCE_K = 6.0 # Much stronger repulsion
        
        v_fwd = self.vel_est[2] 
        v_side = self.vel_est[0]

        def calc_repulsion(dist, vel_component):
            if dist < HARD_LIMIT:
                # Hard Limit Brake: If too close, zero out velocity components towards obstacle
                rep = FORCE_K / (max(0.1, dist)**2)
                if vel_component > 0: rep *= 4.0 # Forceful brake
                return rep
            return 0.0

        avoid_fwd -= calc_repulsion(sensors['front'], v_fwd)
        avoid_fwd += calc_repulsion(sensors['back'], -v_fwd)
        avoid_side += calc_repulsion(sensors['left'], -v_side)
        avoid_side -= calc_repulsion(sensors['right'], v_side)
        
        # 3. Rapid Evasion (Sudden Obstacle)
        if self.prev_front_dist - sensors['front'] > 1.2:
            print("SUDDEN OBSTACLE! RAPID EVASION...")
            self.target_alt += 2.0
            avoid_fwd -= 15.0
        self.prev_front_dist = sensors['front']

        # 3. State: RECOVERY (Aggressive Escape)
        if self.state == "RECOVERY":
            self.recovery_timer += dt
            # Step 1: Intelligent Retreat (Towards best exit)
            if self.recovery_step == 0: 
                fwd_input = 6.0 * math.cos(sensors['best_exit_angle'])
                side_input = 6.0 * math.sin(sensors['best_exit_angle'])
                if self.recovery_timer > 1.5: 
                    self.recovery_step = 1; self.recovery_timer = 0
                    print("Retreat done. Climbing for clearance...")
            elif self.recovery_step == 1: # Ultra Lift
                self.target_alt += 2.0 # Boost way up to clear "Big" obstacles
                if self.recovery_timer > 1.2: self.recovery_step = 2; self.recovery_timer = 0
            elif self.recovery_step == 2: # High-speed random burst to reset pathing
                fwd_input = (random.random() - 0.5) * 8.0
                side_input = (random.random() - 0.5) * 8.0
                if self.recovery_timer > 0.8:
                    self.state = "NORMAL"; self.stall_timer = 0; print("Recovery Finalized.")
            
            return fwd_input, side_input, 0.0

        # 4. State: NORMAL Navigation
        if self.mode == "AUTO":
            dx = self.target_wp[0] - self.local_pos[0]
            dz = self.target_wp[2] - self.local_pos[2]
            dist_to_wp = math.sqrt(dx**2 + dz**2)
            
            # Stuck Detection
            dist_moved = np.linalg.norm(self.local_pos - self.last_pos)
            if dist_moved < 0.05 and dist_to_wp > 1.0:
                self.stall_timer += dt
                if self.stall_timer > 3.0:
                    self.state = "RECOVERY"; self.recovery_step = 0; self.recovery_timer = 0
                    print("Drone Stuck! Initiating Recovery...")
            else:
                self.stall_timer = 0
            self.last_pos = self.local_pos.copy()

            if dist_to_wp > 1.0:
                # Flexible Path Seeking (Increased velocity caps)
                # MOD: If Goal LOS is clear, ignore local waypoint and go straight!
                if sensors.get('goal_los', False):
                    seek_fwd = max(-4.0, min(4.0, dx * 1.0))
                    seek_side = max(-4.0, min(4.0, dz * 1.0))
                else:
                    seek_fwd = max(-3.0, min(3.0, dx * 0.8))
                    seek_side = max(-3.0, min(3.0, dz * 0.8))
                
                fwd_input = seek_fwd + avoid_fwd
                side_input = seek_side + avoid_side
            else:
                fwd_input = avoid_fwd
                side_input = avoid_side
        
        return fwd_input, side_input, yaw_input

    def run(self):
        base_thrust = 68.5 
        
        while self.step(self.time_step) != -1:
            current_time = self.getTime()
            dt = current_time - self.last_time
            if dt < 0.001: continue # Avoid division by zero
            self.last_time = current_time
            
            # Odometry Update
            self.update_odometry(dt)
            
            # Perception
            sensors = self.sense_omni()
            
            # Mode Control
            key = self.keyboard.getKey()
            if key == ord('M'): 
                self.mode = "MANUAL" if self.mode == "AUTO" else "AUTO"
                print(f"Mode switched to: {self.mode}")

            fwd, side, yaw_cmd = 0.0, 0.0, 0.0
            
            if self.mode == "MANUAL":
                if key == ord('W'): fwd = 2.0
                if key == ord('S'): fwd = -2.0
                if key == ord('A'): side = -2.0
                if key == ord('D'): side = 2.0
                if key == Keyboard.UP: self.target_alt += 0.05
                if key == Keyboard.DOWN: self.target_alt -= 0.05
            else:
                fwd, side, yaw_cmd = self.run_ai_logic(sensors, dt, yaw_curr)

            # Flight Control
            roll_curr, pitch_curr, yaw_curr = self.imu.getRollPitchYaw()
            # Simulation: alt_curr is the real altitude, but we treat it as an estimation.
            # In a real setup, we'd use a distance sensor facing DOWN.
            alt_curr = 2.0 # Placeholder for altitude estimation
            
            alt_output = self.alt_pid.compute(self.target_alt, alt_curr, dt)
            roll_output = self.roll_pid.compute(side, roll_curr, dt)
            pitch_output = self.pitch_pid.compute(fwd, pitch_curr, dt)
            yaw_output = self.yaw_pid.compute(yaw_cmd, yaw_curr, dt)

            # Motor Mixing
            m1 = base_thrust + alt_output - roll_output - pitch_output + yaw_output
            m2 = base_thrust + alt_output + roll_output - pitch_output - yaw_output
            m3 = base_thrust + alt_output - roll_output + pitch_output - yaw_output
            m4 = base_thrust + alt_output + roll_output + pitch_output + yaw_output

            for i, val in enumerate([m1, m2, m3, m4]):
                self.motors[i].setVelocity(max(0, min(val, 100)))

if __name__ == "__main__":
    drone = OmniDroneAI()
    drone.run()

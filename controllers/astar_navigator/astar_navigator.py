"""astar_navigator controller."""
import math
import json
import os
from controller import Robot

TIME_STEP = 8

class AStarNavigator(Robot):
    def __init__(self):
        super(AStarNavigator, self).__init__()
        
        # Motors
        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        
        self.motors = [self.front_left_motor, self.front_right_motor, self.rear_left_motor, self.rear_right_motor]
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
            
        # Sensors
        self.gps = self.getDevice("gps")
        self.gps.enable(TIME_STEP)
        self.compass = self.getDevice("compass")
        if self.compass: self.compass.enable(TIME_STEP)
        self.lidar = self.getDevice("lidar") 
        if self.lidar:
            self.lidar.enable(TIME_STEP)
            self.lidar.enablePointCloud()
        
        # State Variables
        self.target_altitude = 2.5
        self.waypoints = []
        self.current_waypoint = 0
        self.load_dynamic_path()
        
        # Recovery & Performance
        self.stall_timer = 0.0
        self.last_pos = [0.0, 0.0, 0.0]
        self.diagnostic_timer = 0.0
        self.ready_to_fly = False
        self.recovery_mode = False
        self.recovery_timer = 0.0
        self.best_recovery_angle = 0.0
        self.prev_front_dist = 10.0

    def load_dynamic_path(self):
        data_path = "../../astar_data.json"
        if os.path.exists(data_path):
            try:
                with open(data_path, 'r') as f:
                    data = json.load(f)
                    path = data.get('path', [])
                    self.waypoints = [(p['x'] - 9.5, p['z'] - 9.5) for p in path]
                    print(f"Loaded {len(self.waypoints)} waypoints.")
            except: pass
        if not self.waypoints: self.waypoints = [(0, 0), (5, 5)]

    def get_heading(self):
        if not self.compass: return 0.0
        north = self.compass.getValues()
        return math.atan2(north[0], north[2])

    def sense_omni(self):
        sensors = {'f': 10.0, 'b': 10.0, 'l': 10.0, 'r': 10.0, 'goal_los': False}
        if self.lidar:
            ranges = self.lidar.getRangeImage()
            num = len(ranges)
            if num > 0:
                arc = num // 6 # 120 deg
                mid = num // 2
                sensors['f'] = min(ranges[mid-arc : mid+arc])
                sensors['l'] = min(ranges[mid+num//4-arc : mid+num//4+arc])
                sensors['r'] = min(ranges[mid-num//4-arc : mid-num//4+arc])
                sensors['b'] = min(min(ranges[:arc]), min(ranges[-arc:]))
                
                if len(self.waypoints) > 0:
                    gx, gz = self.waypoints[-1]
                    px, _, pz = self.gps.getValues()
                    dist_g = math.sqrt((gx-px)**2 + (gz-pz)**2)
                    sensors['goal_los'] = (sensors['f'] > dist_g * 0.9)
        return sensors

    def run(self):
        print("AutoNAV Intelligent Brain Launching...")
        k_p_alt, k_i_alt, k_d_alt = 20.0, 0.5, 10.0
        integral_alt, prev_error_alt = 0.0, 0.0
        base_thrust = 68.5 
        
        while self.step(TIME_STEP) != -1:
            dt = TIME_STEP / 1000.0
            if self.current_waypoint >= len(self.waypoints): break
            
            pos = self.gps.getValues()
            if math.isnan(pos[1]): continue
            
            # Altitude Control
            error_alt = self.target_altitude - pos[1]
            integral_alt += error_alt * dt
            derivative_alt = (error_alt - prev_error_alt) / dt
            prev_error_alt = error_alt
            v_in = (k_p_alt * error_alt) + (k_i_alt * integral_alt) + (k_d_alt * derivative_alt)
            
            # Waypoint Targeting
            wp_x, wp_z = self.waypoints[self.current_waypoint]
            wp_dx, wp_dz = wp_x - pos[0], wp_z - pos[2]
            dist_to_wp = math.sqrt(wp_dx**2 + wp_dz**2)
            
            # Stuck Recovery Logic
            dist_moved = math.sqrt((pos[0]-self.last_pos[0])**2 + (pos[2]-self.last_pos[2])**2)
            if dist_moved < 0.015 and dist_to_wp > 1.2 and self.ready_to_fly and not self.recovery_mode:
                self.stall_timer += dt
                if self.stall_timer > 1.5:
                    print("STUCK! Finding another path...")
                    self.recovery_mode = True
                    self.recovery_timer = 0.0
            else: self.stall_timer = 0.0
            self.last_pos = [pos[0], pos[1], pos[2]]
            
            r_p, r_r, r_y = 0.0, 0.0, 0.0
            if self.recovery_mode:
                self.recovery_timer += dt
                if self.recovery_timer < 0.8: # Back & Up
                    self.target_altitude = 5.5
                    r_p = -25.0
                elif self.recovery_timer < 2.2: # Scan for Space
                    r_y = 18.0
                    ranges = self.lidar.getRangeImage() if self.lidar else []
                    if len(ranges) > 0:
                        max_idx = ranges.index(max(ranges))
                        self.best_recovery_angle = self.get_heading() + (max_idx - len(ranges)/2.0) * (math.pi/len(ranges))
                elif self.recovery_timer < 3.5: # Burst into Gap
                    h_err = (self.best_recovery_angle - self.get_heading() + math.pi) % (2*math.pi) - math.pi
                    r_y, r_p = 6.0 * h_err, 35.0
                else: 
                    self.recovery_mode = False
                    self.current_waypoint = min(len(self.waypoints)-1, self.current_waypoint + 6)
            
            # Launch Cycle
            if not self.ready_to_fly:
                self.diagnostic_timer += dt
                if self.diagnostic_timer > 0.5: self.ready_to_fly = True
                
            # Perception & Avoidance
            sensors = self.sense_omni()
            SAFE = 2.5
            avoid_p, avoid_r = 0.0, 0.0
            speed_mult = min(1.0, (sensors['f'] / SAFE)**2)
            
            if sensors['f'] < SAFE: avoid_p -= 22.0 / (max(0.1, sensors['f'])**2)
            if sensors['l'] < SAFE: avoid_r += 12.0 / (max(0.1, sensors['l'])**2)
            if sensors['r'] < SAFE: avoid_r -= 12.0 / (max(0.1, sensors['r'])**2)
            
            # Skipping logic
            if sensors['goal_los']: self.current_waypoint = len(self.waypoints) - 1
            elif dist_to_wp < 3.0 and not self.recovery_mode: self.current_waypoint += 1
            
            # Control Mixing
            lookahead = min(self.current_waypoint + 6, len(self.waypoints)-1)
            tx, tz = self.waypoints[lookahead]
            dx, dz = tx - pos[0], tz - pos[2]
            ldist = math.sqrt(dx*dx+dz*dz)
            h_err = (math.atan2(dx, dz) - self.get_heading() + math.pi) % (2*math.pi) - math.pi
            
            yaw_in = 3.0 * h_err + r_y
            pitch_in = (min(8.0, ldist * 2.5) * speed_mult) + avoid_p + r_p
            roll_in = avoid_r + r_r
            
            # Final Motors
            m1 = base_thrust + v_in - roll_in - pitch_in + yaw_in
            m2 = base_thrust + v_in + roll_in - pitch_in - yaw_in
            m3 = base_thrust + v_in - roll_in + pitch_in - yaw_in
            m4 = base_thrust + v_in + roll_in + pitch_in + yaw_in
            
            self.front_left_motor.setVelocity(min(100.0, max(0.0, float(m1))))
            self.front_right_motor.setVelocity(min(100.0, max(0.0, float(m2))))
            self.rear_left_motor.setVelocity(min(100.0, max(0.0, float(m3))))
            self.rear_right_motor.setVelocity(min(100.0, max(0.0, float(m4))))

if __name__ == '__main__':
    drone = AStarNavigator()
    drone.run()

from controller import Robot, Motor, InertialUnit, GPS, Gyro, Compass, Lidar, Camera, Display
import numpy as np

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

class DroneOrchestrator(Robot):
    def __init__(self):
        super(DroneOrchestrator, self).__init__()
        self.time_step = int(self.getBasicTimeStep())
        
        # Motors
        self.motors = [
            self.getDevice("m1_motor"), # Front Right
            self.getDevice("m2_motor"), # Front Left
            self.getDevice("m3_motor"), # Rear Right
            self.getDevice("m4_motor")  # Rear Left
        ]
        for m in self.motors:
            m.setPosition(float('inf'))
            m.setVelocity(1.0)

        # Sensors
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(self.time_step)
        self.gps = self.getDevice("gps")
        self.gps.enable(self.time_step)
        self.gyro = self.getDevice("gyro")
        self.gyro.enable(self.time_step)
        self.lidar = self.getDevice("lidar")
        self.lidar.enable(self.time_step)
        self.camera = self.getDevice("camera")
        self.camera.enable(self.time_step)
        self.keyboard = self.getKeyboard()
        self.keyboard.enable(self.time_step)
        
        # PID Controllers (tuned for 8ms)
        self.alt_pid = PID(1.0, 0.05, 0.5)
        self.pitch_pid = PID(0.1, 0, 0.05)
        self.roll_pid = PID(0.1, 0, 0.05)
        
        # State
        self.target_alt = 1.0 # Default takeoff height
        self.waypoints = [(5, 5), (10, -5), (15, 0)] # Example course
        self.wp_idx = 0
        self.is_crashed = False

    def run_perception_agent(self):
        """Processes Lidar and Camera for SLAM / Avoidance"""
        if self.is_crashed: return "CRASHED"
        
        point_cloud = self.lidar.getHorizontalResolution()
        ranges = self.lidar.getRangeImage()
        dist_min = min(ranges)
        
        if dist_min < 0.2: # Instant accident threshold
            self.is_crashed = True
            return "CRASHED"
        
        # Safety Bubble Check (Central 60 degrees)
        center_win = ranges[150:210]
        if min(center_win) < 0.8:
            return "OBSTACLE_DETECTED"
        return "CLEAR"

    def run_navigation_agent(self, status):
        """Strategic target seeking with dual Keyboard/Auto modes"""
        if status == "CRASHED":
            return 0.0, 0.0, 0.0 # fwd, yaw, alt_mod
            
        # 1. Manual Keyboard Override
        key = self.keyboard.getKey()
        if key != -1:
            fwd, yaw, alt_mod = 0.0, 0.0, 0.0
            if key == ord('W') or key == self.keyboard.UP: fwd = 2.0
            if key == ord('S') or key == self.keyboard.DOWN: fwd = -2.0
            if key == ord('A') or key == self.keyboard.LEFT: yaw = 1.0
            if key == ord('D') or key == self.keyboard.RIGHT: yaw = -1.0
            if key == self.keyboard.SHIFT: alt_mod = 1.0
            if key == self.keyboard.CONTROL: alt_mod = -1.0
            return fwd, yaw, alt_mod

        # 2. Automated Waypoint Following
        if status == "OBSTACLE_DETECTED":
            return 0.0, 2.0, 0.0
            
        if self.wp_idx < len(self.waypoints):
            target = self.waypoints[self.wp_idx]
            pos = self.gps.getValues()
            dist = np.sqrt((target[0] - pos[0])**2 + (target[1] - pos[2])**2)
            if dist < 0.5:
                self.wp_idx += 1
                print(f"Reached Waypoint {self.wp_idx}")
            
            target_heading = np.arctan2(target[1] - pos[2], target[0] - pos[0])
            _, _, yaw_angle = self.imu.getRollPitchYaw()
            heading_err = target_heading - yaw_angle
            return 1.5, heading_err * 0.5, 0.0
            
        return 0.0, 0.0, 0.0

    def run_flight_control_agent(self, fwd, yaw, alt_mod):
        """Low-level PID and motor mixing"""
        dt = self.time_step / 1000.0
        
        if self.is_crashed:
            # Gravity: Cut all motors instantly
            for m in self.motors: m.setVelocity(0.0)
            return

        # Current State
        roll, pitch, yaw_angle = self.imu.getRollPitchYaw()
        alt = self.gps.getValues()[1]
        
        # Apply Altitude Adjustment
        self.target_alt += alt_mod * 0.1
        if self.target_alt < 0.1: self.target_alt = 0.1
        
        # PID Outputs
        v_alt = self.alt_pid.compute(self.target_alt - alt, dt)
        v_pitch = self.pitch_pid.compute(fwd - pitch, dt)
        v_roll = self.roll_pid.compute(0 - roll, dt)
        
        k_hover = 68.5 
        
        m1 = k_hover + v_alt + v_pitch + v_roll - yaw
        m2 = k_hover + v_alt + v_pitch - v_roll + yaw
        m3 = k_hover + v_alt - v_pitch + v_roll + yaw
        m4 = k_hover + v_alt - v_pitch - v_roll - yaw
        
        for m, v in zip(self.motors, [m1, m2, m3, m4]):
            m.setVelocity(max(0, min(v, 100)))

    def run(self):
        while self.step(self.time_step) != -1:
            status = self.run_perception_agent()
            fwd, yaw, alt_mod = self.run_navigation_agent(status)
            self.run_flight_control_agent(fwd, yaw, alt_mod)

if __name__ == "__main__":
    controller = DroneOrchestrator()
    controller.run()

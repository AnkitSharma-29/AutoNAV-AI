"""astar_navigator controller."""
import math
from controller import Robot

TIME_STEP = 8

class AStarNavigator(Robot):
    def __init__(self):
        super(AStarNavigator, self).__init__()
        
        self.front_left_motor = self.getDevice("front left propeller")
        self.front_right_motor = self.getDevice("front right propeller")
        self.rear_left_motor = self.getDevice("rear left propeller")
        self.rear_right_motor = self.getDevice("rear right propeller")
        
        self.motors = [
            self.front_left_motor,
            self.front_right_motor,
            self.rear_left_motor,
            self.rear_right_motor
        ]
        
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)
            
        self.gps = self.getDevice("gps")
        self.gps.enable(TIME_STEP)
        
        self.compass = self.getDevice("compass")
        if self.compass:
            self.compass.enable(TIME_STEP)
            
        self.lidar = self.getDevice("lidar") 
        if self.lidar:
            self.lidar.enable(TIME_STEP)
            self.lidar.enablePointCloud()
        
        self.target_altitude = 2.0
        # Hardcoded A* waypoints for 2D maze navigation demo
        self.waypoints = [
            (0, 0),
            (2, 5),
            (7, 8),
            (12, 10),
            (15, 15)
        ]
        self.current_waypoint = 0

    def get_heading(self):
        # Calculate heading in radians from compass
        if not self.compass: return 0.0
        north = self.compass.getValues()
        rad = math.atan2(north[0], north[2])
        return rad

    def run(self):
        print("Drone A* Navigator Starting...")
        
        k_p_alt = 20.0
        k_i_alt = 0.5
        k_d_alt = 10.0
        integral_alt = 0.0
        prev_error_alt = 0.0
        base_thrust = 68.5 
        
        while self.step(TIME_STEP) != -1:
            if self.current_waypoint >= len(self.waypoints):
                print("Goal Reached!")
                break
                
            pos = self.gps.getValues()
            if math.isnan(pos[1]):
                continue
                
            # Altitude PID
            altitude = pos[1]
            error_alt = self.target_altitude - altitude
            integral_alt += error_alt * (TIME_STEP / 1000.0)
            derivative_alt = (error_alt - prev_error_alt) / (TIME_STEP / 1000.0)
            prev_error_alt = error_alt
            vertical_input = (k_p_alt * error_alt) + (k_i_alt * integral_alt) + (k_d_alt * derivative_alt)
            
            # 2D Waypoint Navigation
            target_x, target_z = self.waypoints[self.current_waypoint]
            dx = target_x - pos[0]
            dz = target_z - pos[2]
            dist = math.sqrt(dx*dx + dz*dz)
            
            if dist < 1.0:
                self.current_waypoint += 1
                continue
                
            # Simple proportional navigation (Pitch/Roll)
            target_heading = math.atan2(dx, dz)
            current_heading = self.get_heading()
            
            # Heading error
            heading_err = target_heading - current_heading
            # Normalize to -pi to pi
            heading_err = (heading_err + math.pi) % (2 * math.pi) - math.pi
            
            yaw_input = 2.0 * heading_err
            pitch_input = min(2.0, dist) # Move forward
            roll_input = 0.0
            
            # Motor mixing
            m1 = base_thrust + vertical_input - roll_input - pitch_input + yaw_input
            m2 = base_thrust + vertical_input + roll_input - pitch_input - yaw_input
            m3 = base_thrust + vertical_input - roll_input + pitch_input - yaw_input
            m4 = base_thrust + vertical_input + roll_input + pitch_input + yaw_input
            
            MAX_SPEED = 100.0
            self.front_left_motor.setVelocity(min(MAX_SPEED, max(0, m1)))
            self.front_right_motor.setVelocity(min(MAX_SPEED, max(0, m2)))
            self.rear_left_motor.setVelocity(min(MAX_SPEED, max(0, m3)))
            self.rear_right_motor.setVelocity(min(MAX_SPEED, max(0, m4)))

if __name__ == '__main__':
    drone = AStarNavigator()
    drone.run()

"""pid_altitude_controller controller."""
import math
from controller import Robot

TIME_STEP = 8

class DroneController(Robot):
    def __init__(self):
        super(DroneController, self).__init__()
        
        # Initialize motors
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
            
        # Initialize Sensors (GPS for Altitude, Lidar for Obstacles)
        self.gps = self.getDevice("gps")
        self.gps.enable(TIME_STEP)
        
        self.imu = self.getDevice("inertial unit")
        self.imu.enable(TIME_STEP)
        
        self.lidar = self.getDevice("lidar") # Added to custom slot in wbt
        if self.lidar:
            self.lidar.enable(TIME_STEP)
            self.lidar.enablePointCloud()
        
        # PID Variables
        self.target_altitude = 2.0
        self.integral_error = 0.0
        self.prev_error = 0.0

    def run(self):
        print("Drone PID Controller Starting...")
        
        # PID Constants
        k_p = 20.0
        k_i = 0.5
        k_d = 10.0
        
        base_thrust = 68.5 # Hover thrust baseline for Mavic 2
        
        while self.step(TIME_STEP) != -1:
            # 1. Measure Altitude
            altitude = self.gps.getValues()[1]
            if math.isnan(altitude):
                continue
                
            # 2. Compute PID for altitude
            error = self.target_altitude - altitude
            self.integral_error += error * (TIME_STEP / 1000.0)
            derivative_error = (error - self.prev_error) / (TIME_STEP / 1000.0)
            self.prev_error = error
            
            vertical_input = (k_p * error) + (k_i * self.integral_error) + (k_d * derivative_error)
            
            # 3. Obstacle Avoidance (Lidar reading)
            pitch_input = 0.0
            roll_input = 0.0
            
            if self.lidar:
                range_image = self.lidar.getRangeImage()
                if range_image:
                    # Very simple avoidance: Check front cone
                    front_dist = min(range_image[240:270]) # Roughly front rays if 512 total
                    
                    if front_dist < 2.0:
                        # Obstacle in front! Pitch backward
                        pitch_input = -2.0
                    else:
                        # Move forward autonomously
                        pitch_input = 0.5
            
            # 4. Motor Mixing Algorithm
            m1 = base_thrust + vertical_input - roll_input - pitch_input
            m2 = base_thrust + vertical_input + roll_input - pitch_input
            m3 = base_thrust + vertical_input - roll_input + pitch_input
            m4 = base_thrust + vertical_input + roll_input + pitch_input
            
            # Limit velocities
            MAX_SPEED = 100.0
            
            self.front_left_motor.setVelocity(min(MAX_SPEED, max(0, m1)))
            self.front_right_motor.setVelocity(min(MAX_SPEED, max(0, m2)))
            self.rear_left_motor.setVelocity(min(MAX_SPEED, max(0, m3)))
            self.rear_right_motor.setVelocity(min(MAX_SPEED, max(0, m4)))

if __name__ == '__main__':
    drone = DroneController()
    drone.run()

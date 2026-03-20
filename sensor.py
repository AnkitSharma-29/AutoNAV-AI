import math
import numpy as np

class SimulatedLiDAR:
    def __init__(self, environment, max_range=10.0, fov=360, rays=36):
        self.env = environment
        self.max_range = max_range
        self.fov = fov
        self.rays = rays

    def scan(self, drone_x, drone_z, drone_heading):
        """
        Simulates a 2D LiDAR scan.
        Returns a list of distances at specific angles relative to the drone.
        """
        scan_data = []
        angle_step = self.fov / self.rays
        
        for i in range(self.rays):
            angle_deg = (drone_heading - (self.fov/2)) + (i * angle_step)
            angle_rad = math.radians(angle_deg)
            
            # Raycast
            distance = self.max_range
            step_size = 0.5
            
            for d in np.arange(0, self.max_range, step_size):
                test_x = drone_x - (math.sin(angle_rad) * d)
                test_z = drone_z - (math.cos(angle_rad) * d)
                
                if self.env.is_obstacle(test_x, test_z):
                    distance = d
                    break
                    
            scan_data.append({
                'angle': angle_deg,
                'distance': distance
            })
            
        return scan_data

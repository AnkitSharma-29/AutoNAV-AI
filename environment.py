import numpy as np

class Environment:
    def __init__(self, size=20, obstacle_density=0.2):
        self.size = size
        self.grid = np.zeros((size, size))
        self.grid_resolution = 1.0 # 1 meter per cell
        self.generate_obstacles(obstacle_density)

    def generate_obstacles(self, density):
        # Create some random obstacles, leaving borders empty
        for i in range(1, self.size - 1):
            for j in range(1, self.size - 1):
                if np.random.random() < density:
                    self.grid[i, j] = 1 # 1 is obstacle

        # Ensure start (0,0) and some basic paths are free
        self.grid[0:2, 0:2] = 0
        self.grid[self.size//2, :] = 0  # Clear a tunnel

    def is_obstacle(self, x, z):
        """Check if a continuous coordinate is in an obstacle cell."""
        grid_x = int(round(x / self.grid_resolution))
        grid_z = int(round(z / self.grid_resolution))
        
        # Out of bounds counts as obstacle (wall)
        if grid_x < 0 or grid_x >= self.size or grid_z < 0 or grid_z >= self.size:
            return True
            
        return self.grid[grid_z, grid_x] == 1

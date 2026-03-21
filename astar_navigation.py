import numpy as np
import matplotlib.pyplot as plt
import heapq
import json
import argparse
import random

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0  # Cost from start to current
        self.h = 0  # Heuristic (estimated cost to goal)
        self.f = 0  # Total cost

    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f

def astar(grid, start, end):
    # Weighted A* for faster discovery
    weight = 3.0
    
    # Create start and end nodes
    start_node = Node(start, None)
    end_node = Node(end, None)

    # Initialize open and closed lists
    open_list = []
    closed_list = set()

    # Add start node to open list
    heapq.heappush(open_list, start_node)

    # Loop until the goal is found
    while open_list:
        # Get the current node
        current_node = heapq.heappop(open_list)
        closed_list.add(current_node.position)

        # Found the goal
        if current_node.position == end_node.position:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1] # Return reversed path

        # Generate children
        children = []
        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]: # 8-way movement
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            # Within range
            if node_position[0] > (len(grid) - 1) or node_position[0] < 0 or node_position[1] > (len(grid[len(grid)-1]) - 1) or node_position[1] < 0:
                continue

            # Check if wall/obstacle
            if grid[node_position[0]][node_position[1]] != 0:
                continue

            # Create new node
            new_node = Node(node_position, current_node)
            children.append(new_node)

        # Loop through children
        for child in children:
            if child.position in closed_list:
                continue

            # Calculate costs
            # Octile distance for 8-way movement
            dx = abs(child.position[0] - end_node.position[0])
            dy = abs(child.position[1] - end_node.position[1])
            
            step_dx = abs(child.position[0] - current_node.position[0])
            step_dy = abs(child.position[1] - current_node.position[1])
            step_cost = 1.414 if (step_dx == 1 and step_dy == 1) else 1.0
            
            child.g = current_node.g + step_cost
            child.h = (dx + dy) + (1.414 - 2) * min(dx, dy)
            # Ultra-weighted A* (Weight = 10.0) for near-instant execution
            child.f = child.g + (10.0 * child.h)

            # Is child already in open list with lower cost?
            if any(open_node.position == child.position and child.g > open_node.g for open_node in open_list):
                continue

            # Add to open list
            heapq.heappush(open_list, child)

    return None # No path found

def run_navigation_simulation():
    parser = argparse.ArgumentParser()
    parser.add_argument("--density", type=float, default=0.2)
    parser.add_argument("--start", type=str, default="0,0")
    parser.add_argument("--goal", type=str, default="19,19")
    parser.add_argument("--manual_obs", type=str, default="") # "r,c;r,c;..."
    args = parser.parse_args()

    # Define grid size
    grid_size = 20
    grid = np.zeros((grid_size, grid_size))
    
    # Parse Start/Goal
    try:
        s_coords = [int(float(x)) for x in args.start.split(',')]
        g_coords = [int(float(x)) for x in args.goal.split(',')]
        # Start/Goal are (x, z) from web, map to (col, row) in A*
        start = (s_coords[1], s_coords[0]) 
        goal = (g_coords[1], g_coords[0])
    except:
        start = (0, 0)
        goal = (19, 19)

    # Add manual obstacles
    if args.manual_obs:
        obs_list = args.manual_obs.split(';')
        for obs in obs_list:
            if obs:
                try:
                    c, r = [int(float(x)) for x in obs.split(',')]
                    if 0 <= r < grid_size and 0 <= c < grid_size:
                        grid[r, c] = 1
                except: continue

    # Define a safe zone around start and goal to prevent random obstacles from instantly failing A*
    def is_near_point(r, c, pr, pc):
        return abs(r - pr) <= 1 and abs(c - pc) <= 1

    # Add random obstacles based on density
    for r in range(grid_size):
        for c in range(grid_size):
            if grid[r, c] == 0:
                # Do not place random obstacles on or immediately adjacent to Start/Goal
                if is_near_point(r, c, start[0], start[1]) or is_near_point(r, c, goal[0], goal[1]):
                    continue
                if random.random() < args.density:
                    grid[r, c] = 1
    
    print(f"Calculating path from {start} to {goal}...")
    path = astar(grid, start, goal)

    if path:
        print("Path found!")
        # Export JSON data for 3D visualization
        # Note: In 3D engine, x is col, z is row
        traj_data = [{"x": float(p[1]), "y": 0.5, "z": float(p[0])} for p in path]
        
        # Add obstacle data
        obs_data = []
        for r in range(grid_size):
            for c in range(grid_size):
                if grid[r, c] == 1:
                    obs_data.append({"x": float(c), "y": 0.5, "z": float(r)})
        
        with open('astar_data.json', 'w') as f:
            json.dump({
                "path": traj_data, 
                "obstacles": obs_data, 
                "start": {"x": float(start[1]), "z": float(start[0])}, 
                "goal": {"x": float(goal[1]), "z": float(goal[0])}
            }, f)
        
        # Visualization
        plt.figure(figsize=(10, 10))
        plt.imshow(grid, cmap='Greys', origin='lower')
        path_x = [p[1] for p in path]
        path_y = [p[0] for p in path]
        plt.plot(path_x, path_y, color='red', linewidth=3, label='A* Path')
        plt.scatter(start[1], start[0], color='green', s=200, label='Start')
        plt.scatter(goal[1], goal[0], color='blue', s=200, marker='*', label='Goal')
        plt.title(f'A* Pathfinding (Density: {args.density*100}%)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.savefig('astar_navigation_plot.png')
    else:
        print("No path found.")
        # Even if no path, export the start/goal/obstacles so frontend can show them
        obs_data = []
        for r in range(grid_size):
            for c in range(grid_size):
                if grid[r, c] == 1:
                    obs_data.append({"x": float(c), "y": 0.5, "z": float(r)})
        with open('astar_data.json', 'w') as f:
            json.dump({
                "path": [], 
                "obstacles": obs_data, 
                "start": {"x": float(start[1]), "z": float(start[0])}, 
                "goal": {"x": float(goal[1]), "z": float(goal[0])}
            }, f)

if __name__ == "__main__":
    run_navigation_simulation()

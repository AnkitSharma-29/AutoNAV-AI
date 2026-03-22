import numpy as np
import heapq
import random
import json

# ═══════════════════════════════════════════════════
# A* PATHFINDING LOGIC
# ═══════════════════════════════════════════════════

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = 0 
        self.h = 0 
        self.f = 0 

    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        return self.f < other.f

def astar_core(grid, start, end, weight=3.0):
    start_node = Node(start, None)
    end_node = Node(end, None)
    open_list = []
    closed_list = set()
    # Heap stores (f, node)
    heapq.heappush(open_list, (0, start_node))

    while open_list:
        # Pop node with lowest f
        _, current_node = heapq.heappop(open_list)
        closed_list.add(current_node.position)

        if current_node.position == end_node.position:
            path = []
            curr = current_node
            while curr:
                path.append(curr.position)
                curr = curr.parent
            return path[::-1]

        for new_position in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
            node_pos = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            if node_pos[0] >= len(grid) or node_pos[0] < 0 or node_pos[1] >= len(grid[0]) or node_pos[1] < 0:
                continue
            if grid[node_pos[0]][node_pos[1]] != 0:
                continue

            child = Node(node_pos, current_node)
            if child.position in closed_list:
                continue

            dx = abs(child.position[0] - current_node.position[0])
            dy = abs(child.position[1] - current_node.position[1])
            step_cost = 1.414 if (dx == 1 and dy == 1) else 1.0
            
            child.g = current_node.g + step_cost
            h_dx = abs(child.position[0] - end_node.position[0])
            h_dy = abs(child.position[1] - end_node.position[1])
            child.h = float((h_dx + h_dy) + (1.414 - 2) * min(h_dx, h_dy))
            
            # Tie-breaker
            p_dx = child.position[0] - start_node.position[0]
            p_dy = child.position[1] - start_node.position[1]
            g_dx = end_node.position[0] - start_node.position[0]
            g_dy = end_node.position[1] - start_node.position[1]
            cross = abs(p_dx * g_dy - g_dx * p_dy)
            child.h += cross * 0.001
            
            child.f = child.g + (weight * child.h)

            # Check if this node is already in open_list with a lower g
            if any(child.position == open_node[1].position and child.g >= open_node[1].g for open_node in open_list):
                continue

            heapq.heappush(open_list, (child.f, child))
    return None

def run_astar_sim(params):
    density = float(params.get('density', 0.2))
    start_str = params.get('start', '0,0')
    goal_str = params.get('goal', '19,19')
    manual_obs = params.get('manual_obs', '')

    grid_size = 20
    grid = np.zeros((grid_size, grid_size))
    
    s_coords = [int(float(x)) for x in start_str.split(',')]
    g_coords = [int(float(x)) for x in goal_str.split(',')]
    start = (s_coords[1], s_coords[0]) 
    goal = (g_coords[1], g_coords[0])

    if manual_obs:
        for obs in manual_obs.split(';'):
            if obs:
                c, r = [int(float(x)) for x in obs.split(',')]
                if 0 <= r < grid_size and 0 <= c < grid_size:
                    grid[r, c] = 1

    # Random obstacles
    for r in range(grid_size):
        for c in range(grid_size):
            if grid[r, c] == 0:
                if abs(r - start[0]) <= 1 and abs(c - start[1]) <= 1: continue
                if abs(r - goal[0]) <= 1 and abs(c - goal[1]) <= 1: continue
                if random.random() < density:
                    grid[r, c] = 1

    path = astar_core(grid, start, goal)
    
    resp = {
        "start": {"x": float(start[1]), "z": float(start[0])},
        "goal": {"x": float(goal[1]), "z": float(goal[0])},
        "obstacles": []
    }
    
    for r in range(grid_size):
        for c in range(grid_size):
            if grid[r, c] == 1:
                resp["obstacles"].append({"x": float(c), "y": 0.5, "z": float(r)})
    
    resp["path"] = [{"x": float(p[1]), "y": 0.5, "z": float(p[0])} for p in path] if path else []
    
    # NEW: Write to disk for local Webots support (optional on Vercel)
    try:
        with open('astar_data.json', 'w') as f:
            json.dump(resp, f)
    except:
        pass # Ignore errors on read-only filesystems like Vercel
        
    return resp

# ═══════════════════════════════════════════════════
# PID ALTITUDE LOGIC
# ═══════════════════════════════════════════════════

class PIDController:
    def __init__(self, kp, ki, kd, setpoint):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.setpoint = setpoint
        self.prev_error = 0
        self.integral = 0

    def compute(self, measurement, dt):
        error = self.setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        self.prev_error = error
        return (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)

def run_pid_sim(params):
    target_alt = float(params.get('altitude', 10.0))
    wind = float(params.get('wind', 0.0))
    
    dt, duration = 0.1, 10.0
    steps = int(duration / dt)
    
    pid = PIDController(15.0, 0.5, 8.0, target_alt)
    alt, vel, mass, grav = 0.0, 0.0, 1.0, 9.81
    
    traj = []
    for i in range(steps):
        t = i * dt
        out = pid.compute(alt, dt)
        noise = (random.random() - 0.5) * wind
        thrust = max(0, min(out + (mass * grav) + noise, 40.0))
        
        accel = (thrust - (mass * grav)) / mass
        vel += accel * dt
        alt += vel * dt
        if alt < 0: alt, vel = 0, 0
        
        traj.append({"x": 0, "y": float(alt), "z": 0, "t": float(t)})
    
    result = {"path": traj}
    try:
        with open('pid_data.json', 'w') as f:
            json.dump(traj, f)
    except:
        pass
        
    return result

# ═══════════════════════════════════════════════════
# SLAM LOGIC
# ═══════════════════════════════════════════════════

def run_slam_sim(params):
    noise = float(params.get('noise', 0.1))
    drift = float(params.get('drift', 0.08))
    
    dt, duration = 0.1, 10.0
    steps = int(duration / dt)
    
    true_pos = np.array([0.0, 2.0])
    est_pos = np.array([0.0, 2.0])
    traj = []
    
    for i in range(steps):
        # Move
        vel = np.array([1.0, 0.0])
        actual_vel = vel + np.random.normal(0, noise, 2)
        true_pos += actual_vel * dt
        est_pos += vel * dt
        
        # Drift
        true_pos[1] += (random.random() - 0.5) * drift
        
        # Sense & Correct (every 3 steps)
        if i % 3 == 0:
            d_left = true_pos[1]
            d_right = 4.0 - d_left
            est_pos[1] = (d_left + (4.0 - d_right)) / 2
            
        traj.append({"x": float(true_pos[0]), "y": 1.0, "z": float(true_pos[1])})
        
    result = {"path": traj}
    try:
        with open('slam_data.json', 'w') as f:
            json.dump(traj, f)
    except:
        pass
        
    return result

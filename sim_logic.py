import numpy as np
import heapq
import random
import json
import math
import time

# ═══════════════════════════════════════════════════
# OPTIMIZED NAVIGATION LOGIC (Weighted Lazy Theta*)
# ═══════════════════════════════════════════════════

class Node:
    def __init__(self, position, parent=None):
        self.position = position
        self.parent = parent
        self.g = float('inf')
        self.h = 0.0
        self.f = float('inf')

    def __eq__(self, other):
        return self.position == other.position
    
    def __lt__(self, other):
        if self.f == other.f:
            return self.g > other.g
        return self.f < other.f

def line_of_sight(grid, start, end):
    x0, y0 = start
    x1, y1 = end
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    x, y = x0, y0
    n = 1 + dx + dy
    x_inc = 1 if x1 > x0 else -1
    y_inc = 1 if y1 > y0 else -1
    error = dx - dy
    dx *= 2
    dy *= 2

    for _ in range(n):
        if not (0 <= x < grid.shape[0] and 0 <= y < grid.shape[1]): return False
        if grid[x, y] != 0: return False
        if x == x1 and y == y1: break
        if error > 0:
            x += x_inc
            error -= dy
        elif error < 0:
            y += y_inc
            error += dx
        else:
            x += x_inc
            y += y_inc
            error += dx - dy
    return True

def get_distance(p1, p2):
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

def inflate_grid(grid, radius=1):
    inflated = np.copy(grid)
    r_idx, c_idx = np.where(grid != 0)
    for r, c in zip(r_idx, c_idx):
        for dr in range(-radius, radius + 1):
            for dc in range(-radius, radius + 1):
                nr, nc = r + dr, c + dc
                if 0 <= nr < grid.shape[0] and 0 <= nc < grid.shape[1]:
                    inflated[nr, nc] = 1
    return inflated

def weighted_lazy_theta_star(grid, start, end, weight=3.5):
    start_node = Node(start)
    start_node.g = 0.0
    start_node.parent = start_node
    
    def h(pos):
        dx, dy = abs(pos[0] - end[0]), abs(pos[1] - end[1])
        return (dx + dy) + (1.414 - 2) * min(dx, dy)

    start_node.h = h(start)
    start_node.f = weight * start_node.h
    open_list = [start_node]
    g_score = {start: 0.0}
    closed_set = set()
    start_time = time.time()
    
    while open_list:
        if time.time() - start_time > 0.5: break
        current = heapq.heappop(open_list)
        if current.position in closed_set: continue
        if current.parent.position != current.position:
            if not line_of_sight(grid, current.parent.position, current.position): continue

        closed_set.add(current.position)
        if current.position == end:
            path = []
            curr = current
            while curr.parent != curr:
                path.append(curr.position)
                curr = curr.parent
            path.append(start)
            return path[::-1]

        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
            neighbor_pos = (current.position[0] + dx, current.position[1] + dy)
            if not (0 <= neighbor_pos[0] < grid.shape[0] and 0 <= neighbor_pos[1] < grid.shape[1]): continue
            if grid[neighbor_pos[0], neighbor_pos[1]] != 0: continue
            if neighbor_pos in closed_set: continue

            tentative_g = current.parent.g + get_distance(current.parent.position, neighbor_pos)
            if neighbor_pos not in g_score or tentative_g < g_score[neighbor_pos]:
                g_score[neighbor_pos] = tentative_g
                neighbor = Node(neighbor_pos, current.parent)
                neighbor.g, neighbor.h = tentative_g, h(neighbor_pos)
                neighbor.f = tentative_g + (weight * neighbor.h)
                heapq.heappush(open_list, neighbor)
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
    start, goal = (s_coords[1], s_coords[0]), (g_coords[1], g_coords[0])

    if manual_obs:
        for obs in manual_obs.split(';'):
            if obs:
                c, r = [int(float(x)) for x in obs.split(',')]
                if 0 <= r < grid_size and 0 <= c < grid_size: grid[r, c] = 1

    for r in range(grid_size):
        for c in range(grid_size):
            if grid[r, c] == 0:
                if abs(r - start[0]) <= 1 and abs(c - start[1]) <= 1: continue
                if abs(r - goal[0]) <= 1 and abs(c - goal[1]) <= 1: continue
                if random.random() < density: grid[r, c] = 1

    search_grid = inflate_grid(grid, radius=1)
    search_grid[start[0], start[1]] = 0
    search_grid[goal[0], goal[1]] = 0
    path = weighted_lazy_theta_star(search_grid, start, goal, weight=3.0)
    
    resp = {
        "success": True if path else False,
        "start": {"x": float(start[1]), "z": float(start[0])},
        "goal": {"x": float(goal[1]), "z": float(goal[0])},
        "obstacles": [{"x": float(c), "y": 0.5, "z": float(r)} for r in range(grid_size) for c in range(grid_size) if grid[r, c] == 1],
        "path": [{"x": float(p[1]), "y": 0.5, "z": float(p[0])} for p in path] if path else []
    }
    return resp

# ═══════════════════════════════════════════════════
# PID ALTITUDE LOGIC
# ═══════════════════════════════════════════════════

def run_pid_sim(params):
    target_alt = float(params.get('altitude', 10.0))
    wind = float(params.get('wind', 0.0))
    dt, duration = 0.1, 10.0
    steps = int(duration / dt)
    kp, ki, kd = 15.0, 0.5, 8.0
    alt, vel, mass, grav = 0.0, 0.0, 1.0, 9.81
    prev_error, integral = 0, 0
    traj = []
    for i in range(steps):
        t = i * dt
        error = target_alt - alt
        integral += error * dt
        derivative = (error - prev_error) / dt
        prev_error = error
        out = (kp * error) + (ki * integral) + (kd * derivative)
        noise = (random.random() - 0.5) * wind
        thrust = max(0, min(out + (mass * grav) + noise, 40.0))
        accel = (thrust - (mass * grav)) / mass
        vel += accel * dt
        alt += vel * dt
        if alt < 0: alt, vel = 0, 0
        traj.append({"x": 0, "y": float(alt), "z": 0, "t": float(t)})
    return {"path": traj}

# ═══════════════════════════════════════════════════
# SLAM LOGIC
# ═══════════════════════════════════════════════════

def run_slam_sim(params):
    noise = float(params.get('noise', 0.08))
    drift = float(params.get('drift', 0.04))
    dt, duration = 0.1, 10.0
    steps = int(duration / dt)
    true_pos = np.array([0.0, 2.0])
    traj = []
    history_left, history_right = [], []
    for i in range(steps):
        true_pos += (np.array([1.0, 0.0]) + np.random.normal(0, noise, 2)) * dt
        true_pos[1] += (random.random() - 0.5) * drift
        traj.append({"x": float(true_pos[0]), "y": 1.0, "z": float(true_pos[1])})
    return {"path": traj}

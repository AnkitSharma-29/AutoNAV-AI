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

def line_of_sight(grid, p1, p2):
    """
    Bresenham's line algorithm to check if there's a clear path between p1 and p2.
    """
    x0, y0 = p1
    x1, y1 = p2
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    n = 1 + dx + dy
    x_inc = 1 if x1 > x0 else -1
    y_inc = 1 if y1 > y0 else -1
    error = dx - dy
    dx *= 2
    dy *= 2

    for _ in range(n):
        if grid[x][y] != 0:
            return False
        if error > 0:
            x += x_inc
            error -= dy
        elif error < 0:
            y += y_inc
            error += dx
        else:
            # error == 0, diagonal step
            x += x_inc
            y += y_inc
            error += dx - dy
        if x == x1 and y == y1:
            break
    return True

def get_safety_cost(grid, pos, safety_radius=4):
    """
    Returns a high cost penalty for cells near obstacles.
    Uses inverse-cube for aggressive distance-keeping.
    """
    r, c = pos
    min_dist = safety_radius + 1
    
    for dr in range(-safety_radius, safety_radius + 1):
        for dc in range(-safety_radius, safety_radius + 1):
            nr, nc = r + dr, c + dc
            if 0 <= nr < len(grid) and 0 <= nc < len(grid[0]):
                if grid[nr][nc] != 0:
                    dist = (dr**2 + dc**2)**0.5
                    if dist < min_dist:
                        min_dist = dist
    
    if min_dist <= 1.5: return 200.0   # Near-impassable: too close to wall
    if min_dist <= 2.5: return 80.0    # Very expensive: uncomfortably close
    if min_dist <= safety_radius:
        return 40.0 / (min_dist**3)    # Inverse-cube: gentle falloff at distance
    return 0.0

def astar(grid, start, end, weight=1.5, mode='advanced'):
    """
    Corrected Lazy Theta* with proper LoS fallback.
    When the lazy LoS assumption fails, re-parent through the best closed neighbor.
    """
    start_node = Node(start)
    end_node = Node(end)
    start_node.parent = start_node
    start_node.g = 0
    
    open_list = []
    closed_list = set()
    open_dict = {start: start_node}
    closed_nodes = {start: start_node}  # Track actual Node objects in closed set
    
    heapq.heappush(open_list, start_node)
    
    while open_list:
        current_node = heapq.heappop(open_list)
        if current_node.position in closed_list: continue
        
        # --- LAZY THETA* CORRECTION ---
        if current_node.position != start and mode == 'advanced':
            parent = current_node.parent
            if not line_of_sight(grid, parent.position, current_node.position):
                # LoS failed! Re-parent through best closed-list neighbor
                best_g = float('inf')
                best_parent = None
                for dr, dc in [(0,-1),(0,1),(-1,0),(1,0),(-1,-1),(-1,1),(1,-1),(1,1)]:
                    nb = (current_node.position[0]+dr, current_node.position[1]+dc)
                    if nb in closed_nodes:
                        step = 1.414 if (dr!=0 and dc!=0) else 1.0
                        candidate_g = closed_nodes[nb].g + step + get_safety_cost(grid, current_node.position)
                        if candidate_g < best_g:
                            best_g = candidate_g
                            best_parent = closed_nodes[nb]
                if best_parent:
                    current_node.parent = best_parent
                    current_node.g = best_g
                    current_node.f = current_node.g + (weight * current_node.h)
        # ------------------------------

        closed_list.add(current_node.position)
        closed_nodes[current_node.position] = current_node

        if current_node.position == end_node.position:
            path = []
            curr = current_node
            while curr.parent != curr:
                path.append(curr.position)
                curr = curr.parent
            path.append(start)
            return path[::-1]

        for dr, dc in [(0,-1),(0,1),(-1,0),(1,0),(-1,-1),(-1,1),(1,-1),(1,1)]:
            node_pos = (current_node.position[0]+dr, current_node.position[1]+dc)
            if not (0 <= node_pos[0] < len(grid) and 0 <= node_pos[1] < len(grid[0])): continue
            if grid[node_pos[0]][node_pos[1]] != 0: continue
            if node_pos in closed_list: continue

            parent = current_node.parent
            # LAZY ASSUMPTION: Assume LoS from grandparent
            if mode == 'advanced':
                new_g = parent.g + ((parent.position[0]-node_pos[0])**2 + (parent.position[1]-node_pos[1])**2)**0.5
                new_g += get_safety_cost(grid, node_pos)
                new_parent = parent
            else:
                step_cost = 1.414 if (dr!=0 and dc!=0) else 1.0
                new_g = current_node.g + step_cost
                new_parent = current_node

            if node_pos not in open_dict or new_g < open_dict[node_pos].g:
                child = Node(node_pos, new_parent)
                child.g = new_g
                dx, dy = abs(node_pos[0]-end_node.position[0]), abs(node_pos[1]-end_node.position[1])
                child.h = (dx**2 + dy**2)**0.5
                child.f = child.g + (weight * child.h)
                open_dict[node_pos] = child
                heapq.heappush(open_list, child)
    return None

def run_astar(density=0.2, start_pos=(0, 0), goal_pos=(19, 19), manual_obs_str="", save_files=True, weight=10.0, mode='advanced'):
    # Define grid size
    grid_size = 20
    grid = np.zeros((grid_size, grid_size))
    
    # Start/Goal mapping (x, z) to (row, col)
    start = (start_pos[1], start_pos[0]) 
    goal = (goal_pos[1], goal_pos[0])

    # Add manual obstacles
    if manual_obs_str:
        obs_list = manual_obs_str.split(';')
        for obs in obs_list:
            if obs:
                try:
                    c, r = [int(float(x)) for x in obs.split(',')]
                    if 0 <= r < grid_size and 0 <= c < grid_size:
                        grid[r, c] = 1
                except: continue

    # Define a safe zone around start and goal
    def is_near_point(r, c, pr, pc):
        return abs(r - pr) <= 1 and abs(c - pc) <= 1

    # Add random obstacles
    for r in range(grid_size):
        for c in range(grid_size):
            if grid[r, c] == 0:
                if is_near_point(r, c, start[0], start[1]) or is_near_point(r, c, goal[0], goal[1]):
                    continue
                if random.random() < density:
                    grid[r, c] = 1
    
    path = astar(grid, start, goal, weight=weight, mode=mode)

    result = {
        "success": False,
        "path": [],
        "obstacles": [],
        "start": {"x": float(start[1]), "z": float(start[0])},
        "goal": {"x": float(goal[1]), "z": float(goal[0])}
    }

    # Populate obstacle data
    obs_data = []
    for r in range(grid_size):
        for c in range(grid_size):
            if grid[r, c] == 1:
                obs_data.append({"x": float(c), "y": 0.5, "z": float(r)})
    result["obstacles"] = obs_data

    if path:
        result["success"] = True
        result["path"] = [{"x": float(p[1]), "y": 0.5, "z": float(p[0])} for p in path]
        
        if save_files:
            with open('astar_data.json', 'w') as f:
                json.dump(result, f)
            
            # Visualization
            plt.figure(figsize=(10, 10))
            plt.imshow(grid, cmap='Greys', origin='lower')
            path_x = [p[1] for p in path]
            path_y = [p[0] for p in path]
            plt.plot(path_x, path_y, color='red', linewidth=3, label='A* Path')
            plt.scatter(start[1], start[0], color='green', s=200, label='Start')
            plt.scatter(goal[1], goal[0], color='blue', s=200, marker='*', label='Goal')
            plt.title(f'A* Pathfinding (Density: {density*100}%)')
            plt.legend()
            plt.grid(True, alpha=0.3)
            plt.savefig('astar_navigation_plot.png')
            plt.close()
    else:
        if save_files:
            with open('astar_data.json', 'w') as f:
                json.dump(result, f)

    return result

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--density", type=float, default=0.2)
    parser.add_argument("--start", type=str, default="0,0")
    parser.add_argument("--goal", type=str, default="19,19")
    parser.add_argument("--manual_obs", type=str, default="")
    args = parser.parse_args()

    # Parse Start/Goal for CLI compatibility
    try:
        s_coords = [int(float(x)) for x in args.start.split(',')]
        g_coords = [int(float(x)) for x in args.goal.split(',')]
    except:
        s_coords = [0, 0]
        g_coords = [19, 19]

    run_astar(args.density, tuple(s_coords), tuple(g_coords), args.manual_obs)

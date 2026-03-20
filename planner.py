import heapq

class AStarPlanner:
    def __init__(self, environment):
        self.env = environment

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node):
        x, z = node
        neighbors = [(x+1,z), (x-1,z), (x,z+1), (x,z-1),
                     (x+1,z+1), (x-1,z-1), (x+1,z-1), (x-1,z+1)]
        valid = []
        for nx, nz in neighbors:
            if 0 <= nx < self.env.size and 0 <= nz < self.env.size:
                if self.env.grid[nz, nx] == 0: # Not obstacle
                    valid.append((nx, nz))
        return valid

    def plan(self, start_pos, goal_pos):
        """
        start_pos: (x, z) in meters
        goal_pos: (x, z) in meters
        Returns list of (x, z) waypoints
        """
        start = (int(round(start_pos[0])), int(round(start_pos[1])))
        goal = (int(round(goal_pos[0])), int(round(goal_pos[1])))
        
        frontier = []
        heapq.heappush(frontier, (0, start))
        came_from = {start: None}
        cost_so_far = {start: 0}

        while frontier:
            _, current = heapq.heappop(frontier)

            if current == goal:
                break

            for nxt in self.get_neighbors(current):
                new_cost = cost_so_far[current] + 1
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    priority = new_cost + self.heuristic(nxt, goal)
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = current

        # Reconstruct path
        path = []
        if goal in came_from:
            curr = goal
            while curr != start:
                path.append(curr)
                curr = came_from[curr]
            path.append(start)
            path.reverse()
            
        return path

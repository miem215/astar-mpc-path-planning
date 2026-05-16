import numpy as np

class GridMap:
    """Handles the discretization of the environment into a grid."""
    DIRS4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    def __init__(self, rows, cols):
        self.rows = rows
        self.cols = cols
        self.grid = np.zeros((rows, cols), dtype=np.uint8)

    @classmethod
    def default_setup(cls):
        """Creates the barriers used in the project."""
        g = cls(36, 56)
        
        for r in range(6, 30): g.grid[r][18] = 1
        for c in range(18, 40): 
            g.grid[6][c] = 1
            g.grid[28][c] = 1
        for r in (16, 17, 18, 19, 20): g.grid[r][18] = 0
        for r in range(8, 28): g.grid[r][38] = 1
        for r in (16, 17, 18, 19, 20): g.grid[r][38] = 0
        return g

    def is_free(self, r, c):
        return 0 <= r < self.rows and 0 <= c < self.cols and not self.grid[r][c]

    def get_neighbours(self, r, c):
        return [(r+dr, c+dc) for dr, dc in self.DIRS4 if self.is_free(r+dr, c+dc)]

def run_astar(gmap, start, goal):
    """Discrete Planning: A* Search Implementation."""
    h = lambda r, c: abs(r - goal[0]) + abs(c - goal[1]) # distance to the goal
    open_set = [(h(*start), 0, start)]
    g_score = {start: 0}
    parent = {start: None}
    closed = set()
    explored = []

    while open_set:
        open_set.sort(key=lambda x: x[0])
        _, gcur, node = open_set.pop(0)
        if node in closed: continue
        closed.add(node)
        explored.append(node)
        if node == goal: break
        
        for nb in gmap.get_neighbours(*node):
            if nb in closed: continue
            ng = gcur + 1
            if ng < g_score.get(nb, float('inf')):
                g_score[nb] = ng
                parent[nb] = node
                open_set.append((ng + h(*nb), ng, nb))

    # Path reconstruction
    path, cur = [], goal
    while cur is not None:
        path.append(cur)
        cur = parent.get(cur)
    return path[::-1], explored
import numpy as np
from scipy.optimize import minimize
from dataclasses import dataclass

@dataclass
class DynamicObstacle:
    """Represents a moving obstacle in continuous space."""
    r: float; c: float; vr: float = 0.0; vc: float = 0.18; active: bool = False

    def step(self, gmap):
        if not self.active: return
        nr, nc = self.r + self.vr, self.c + self.vc
        if not (0 < round(nr) < gmap.rows - 1 and 0 < round(nc) < gmap.cols - 1) or gmap.grid[round(nr)][round(nc)]:
            self.vr, self.vc = -self.vr, -self.vc
        else:
            self.r, self.c = nr, nc

class MPCController:
    """Local replanner using Gradient-based optimization."""
    def __init__(self, horizon=5, w_track=3.0, w_obs=8.0):
        self.H = horizon
        self.W_TRACK = w_track
        self.W_OBS = w_obs
        self.R_SAFE = 2.5

    def compute_step(self, pos, ref_path, obs):
        # Finds nearest point on A* path to track
        dists = [np.linalg.norm(pos - p) for p in ref_path]
        idx = min(int(np.argmin(dists)) + 2, len(ref_path) - 1)
        
        ref = np.array([ref_path[min(idx + k, len(ref_path) - 1)] for k in range(self.H)])

        def cost_function(x):
            p_temp = pos.copy()
            total_cost = 0.0
            for k in range(self.H):
                delta = x[2*k: 2*k + 2]
                p_temp = p_temp + delta
                total_cost += self.W_TRACK * np.sum((p_temp - ref[k])**2)
                if obs.active:
                    d = np.linalg.norm(p_temp - np.array([obs.r, obs.c]))
                    if d < self.R_SAFE:
                        total_cost += self.W_OBS * (self.R_SAFE - d)**2
            return total_cost

        res = minimize(cost_function, np.zeros(self.H * 2), method='L-BFGS-B', bounds=[(-1.2, 1.2)] * (self.H * 2))
        return pos + res.x[0:2]
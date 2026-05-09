"""
A* + MPC Path Planning
======================
Personal project combining two areas of study:

  1. A* Path Planning  (Udacity Robotics Software Engineer Nanodegree)
       Finds the optimal global path on a discrete grid using a
       Manhattan-distance heuristic.

  2. Model Predictive Control  (MSc Systems & Control, TU Delft)
       Executes the A* path locally, replanning at every timestep
       when a dynamic obstacle appears mid-mission.

Architecture
------------
Two-layer stack:

    Layer 1 — Global planner (A*)
        Runs once at mission start.
        Finds the shortest collision-free path on the grid.
        Fast and optimal, but static — cannot react to moving obstacles.

    Layer 2 — Local MPC replanner
        Runs every timestep.
        Tracks the A* reference path.
        Avoids the dynamic obstacle in real time by minimising:

            J = W_TRACK  * tracking error to A* reference
              + W_SMOOTH * path roughness (penalise sharp turns)
              + W_OBS    * obstacle proximity penalty

        Only the first step is executed at each timestep —
        the receding-horizon principle.

Connection to prior work
------------------------
Same receding-horizon structure as VTOL-Transition-MPC
(thrust allocation over a 1-second horizon for a 6-DoF drone),
applied here to 2-D robot navigation.

Output
------
Three PNG files saved to the current folder:
    01_astar.png          A* explored cells and global path
    02_mpc_execution.png  MPC trajectory tracking the A* path
    03_comparison.png     A*-only vs A*+MPC side by side

Usage
-----
    python astar_mpc.py

Dependencies
------------
    pip install matplotlib numpy scipy
"""

from __future__ import annotations

import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional

import matplotlib
matplotlib.use('Agg')          # no interactive windows
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from scipy.optimize import minimize


# ══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ══════════════════════════════════════════════════════════════════════════════

ROWS, COLS = 18, 28
START = (9, 2)
GOAL  = (9, 25)

# Dynamic obstacle
DYN_START  = (9, 14)   # position when it first appears
DYN_SPEED  = 0.18      # cells per timestep
DYN_APPEAR = 6         # timestep it becomes active

# MPC parameters
H        = 5     # horizon length (steps)
STEP_MAX = 1.2   # max displacement per MPC step (cell units)
W_TRACK  = 3.0   # weight: track A* reference path
W_SMOOTH = 0.5   # weight: penalise sharp direction changes
W_OBS    = 8.0   # weight: stay away from dynamic obstacle
R_SAFE   = 2.5   # safety radius around obstacle (cells)

# Colours
C = {
    "bg":      "#ffffff",
    "grid":    "#e8e6df",
    "wall":    "#3d3d3a",
    "start":   "#1D9E75",
    "goal":    "#E24B4A",
    "explored":"#B5D4F4",
    "global":  "#378ADD",
    "mpc":     "#EF9F27",
    "robot":   "#534AB7",
    "horizon": "#AFA9EC",
    "dyn_obs": "#F0997B",
}


# ══════════════════════════════════════════════════════════════════════════════
#  OCCUPANCY GRID
# ══════════════════════════════════════════════════════════════════════════════

class GridMap:
    """2-D occupancy grid. 0 = free, 1 = static obstacle."""

    DIRS4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]

    def __init__(self, rows: int, cols: int) -> None:
        self.rows = rows
        self.cols = cols
        self.grid = np.zeros((rows, cols), dtype=np.uint8)

    @classmethod
    def default(cls) -> "GridMap":
        """Two barriers with gaps — robot must navigate through."""
        g = cls(ROWS, COLS)
        for r in range(3, 15):
            g.grid[r][9] = 1
        for c in range(9, 20):
            g.grid[3][c] = 1
            g.grid[14][c] = 1
        for r in (8, 9, 10):
            g.grid[r][9] = 0
        for r in range(4, 14):
            g.grid[r][19] = 1
        for r in (8, 9, 10):
            g.grid[r][19] = 0
        return g

    def free(self, r: int, c: int) -> bool:
        return (0 <= r < self.rows and
                0 <= c < self.cols and
                not self.grid[r][c])

    def neighbours(self, r: int, c: int) -> list[tuple[int, int]]:
        return [(r+dr, c+dc) for dr, dc in self.DIRS4
                if self.free(r+dr, c+dc)]


# ══════════════════════════════════════════════════════════════════════════════
#  LAYER 1 — A* GLOBAL PLANNER
# ══════════════════════════════════════════════════════════════════════════════

def run_astar(gmap: GridMap) -> tuple[list, list]:
    """
    A* Search with Manhattan-distance heuristic.

    f(n) = g(n) + h(n)
      g(n) — steps from start to n
      h(n) — Manhattan distance from n to goal

    Returns (path, explored_cells).
    """
    h = lambda r, c: abs(r - GOAL[0]) + abs(c - GOAL[1])
    open_set = [(h(*START), 0, START)]
    g: dict[tuple, float] = {START: 0}
    parent: dict[tuple, Optional[tuple]] = {START: None}
    closed: set[tuple] = set()
    explored = []

    while open_set:
        open_set.sort(key=lambda x: x[0])
        _, gcur, node = open_set.pop(0)
        if node in closed:
            continue
        closed.add(node)
        explored.append(node)
        if node == GOAL:
            break
        for nb in gmap.neighbours(*node):
            if nb in closed:
                continue
            ng = gcur + 1
            if ng < g.get(nb, float('inf')):
                g[nb] = ng
                parent[nb] = node
                open_set.append((ng + h(*nb), ng, nb))

    if GOAL not in parent:
        return [], explored
    path, cur = [], GOAL
    while cur is not None:
        path.append(cur)
        cur = parent[cur]
    path.reverse()
    return (path if path[0] == START else []), explored


# ══════════════════════════════════════════════════════════════════════════════
#  LAYER 2 — LOCAL MPC REPLANNER
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class DynObs:
    """Moving obstacle. Becomes active at timestep DYN_APPEAR."""
    r:      float
    c:      float
    vr:     float = 0.0
    vc:     float = DYN_SPEED
    active: bool  = False

    def step(self, gmap: GridMap) -> None:
        if not self.active:
            return
        nr, nc = self.r + self.vr, self.c + self.vc
        ri, ci = round(nr), round(nc)
        if (not (0 < ri < gmap.rows - 1 and 0 < ci < gmap.cols - 1)
                or gmap.grid[ri][ci]):
            self.vr, self.vc = -self.vr, -self.vc
        else:
            self.r, self.c = nr, nc


def mpc_step(pos: np.ndarray, ref_path: list,
             ref_idx: int, obs: DynObs) -> np.ndarray:
    """
    One receding-horizon MPC step.

    Minimises J = W_TRACK * tracking error
                + W_SMOOTH * roughness
                + W_OBS   * obstacle penalty
    over H steps, executes only the first step.
    """
    ref = np.array([
        ref_path[min(ref_idx + k, len(ref_path) - 1)]
        for k in range(H)
    ], dtype=float)

    def cost(x: np.ndarray) -> float:
        p = pos.copy()
        total = 0.0
        for k in range(H):
            delta = x[2*k: 2*k + 2]
            p = p + delta
            total += W_TRACK  * float(np.sum((p - ref[k])**2))
            total += W_SMOOTH * float(np.sum(delta**2))
            if obs.active:
                d = float(np.linalg.norm(p - np.array([obs.r, obs.c])))
                if d < R_SAFE:
                    total += W_OBS * (R_SAFE - d)**2
        return total

    bounds = [(-STEP_MAX, STEP_MAX)] * (H * 2)
    result = minimize(cost, np.zeros(H * 2), method='L-BFGS-B',
                      bounds=bounds,
                      options={'maxiter': 60, 'ftol': 1e-4})
    delta    = result.x[0:2]
    next_pos = np.clip(pos + delta, [0, 0], [ROWS - 1, COLS - 1])
    return next_pos


def nearest_idx(pos: np.ndarray, path: list) -> int:
    dists = [math.hypot(pos[0]-p[0], pos[1]-p[1]) for p in path]
    return min(int(np.argmin(dists)) + 2, len(path) - 1)


def run_mpc(gmap: GridMap, astar_path: list,
            obs: DynObs, max_steps: int = 200) -> tuple[list, list]:
    """Full MPC-guided mission. Returns (trajectory, horizons)."""
    pos        = np.array(START, dtype=float)
    trajectory = [pos.copy()]
    horizons   = []

    for t in range(max_steps):
        if t == DYN_APPEAR:
            obs.active = True
        obs.step(gmap)
        idx = nearest_idx(pos, astar_path)
        pos = mpc_step(pos, astar_path, idx, obs)
        trajectory.append(pos.copy())
        horizons.append([
            astar_path[min(idx + k, len(astar_path) - 1)]
            for k in range(H)
        ])
        if math.hypot(pos[0]-GOAL[0], pos[1]-GOAL[1]) < 1.0:
            break

    return trajectory, horizons


# ══════════════════════════════════════════════════════════════════════════════
#  VISUALISATION  (saves PNGs — no interactive windows)
# ══════════════════════════════════════════════════════════════════════════════

def _draw_base(ax: plt.Axes, gmap: GridMap) -> None:
    ax.set_facecolor(C["bg"])
    ax.set_xlim(-0.5, gmap.cols - 0.5)
    ax.set_ylim(-0.5, gmap.rows - 0.5)
    ax.invert_yaxis()
    ax.set_aspect('equal')
    ax.set_xticks([])
    ax.set_yticks([])
    for r in range(gmap.rows + 1):
        ax.axhline(r - 0.5, color=C["grid"], lw=0.4, zorder=0)
    for c in range(gmap.cols + 1):
        ax.axvline(c - 0.5, color=C["grid"], lw=0.4, zorder=0)
    for r in range(gmap.rows):
        for c in range(gmap.cols):
            if gmap.grid[r][c]:
                ax.add_patch(plt.Rectangle(
                    (c-.45, r-.45), .9, .9, color=C["wall"], zorder=1))
    for (r, c), col, lbl in [(START, C["start"], "S"), (GOAL, C["goal"], "G")]:
        ax.add_patch(plt.Rectangle(
            (c-.42, r-.42), .84, .84, color=col, zorder=4, lw=0))
        ax.text(c, r, lbl, color='white', fontsize=7,
                ha='center', va='center', fontweight='bold', zorder=5)


def save_astar(gmap, explored, path) -> None:
    fig, ax = plt.subplots(figsize=(13, 6))
    fig.patch.set_facecolor(C["bg"])
    _draw_base(ax, gmap)
    for (r, c) in explored:
        ax.add_patch(plt.Rectangle(
            (c-.45, r-.45), .9, .9, color=C["explored"], zorder=2, alpha=0.6))
    if path:
        ax.plot([p[1] for p in path], [p[0] for p in path],
                color=C["global"], lw=2.5, solid_capstyle='round', zorder=3)
    _draw_base(ax, gmap)
    ax.set_title(
        f"Layer 1 — A* global planner  |  "
        f"explored {len(explored)} cells  |  path {len(path)} cells",
        fontsize=10, pad=6, color="#2c2c2a")
    ax.legend(handles=[
        mpatches.Patch(color=C["explored"], label="Explored cells"),
        mpatches.Patch(color=C["global"],   label="A* path"),
    ], fontsize=8, loc='upper left')
    plt.tight_layout()
    plt.savefig("./01_astar.png", dpi=150, bbox_inches='tight')
    plt.close()
    print("Saved: 01_astar.png")


def save_mpc(gmap, astar_path, trajectory, horizons, obs) -> None:
    fig, ax = plt.subplots(figsize=(13, 6))
    fig.patch.set_facecolor(C["bg"])
    _draw_base(ax, gmap)
    if astar_path:
        ax.plot([p[1] for p in astar_path], [p[0] for p in astar_path],
                color=C["global"], lw=1.5, linestyle='--',
                zorder=2, alpha=0.6, label="A* global path")
    if horizons:
        hpts = horizons[-1]
        ax.plot([p[1] for p in hpts], [p[0] for p in hpts],
                color=C["horizon"], lw=1, marker='o',
                markersize=3, zorder=3, alpha=0.8, label="MPC horizon")
    if len(trajectory) > 1:
        ax.plot([p[1] for p in trajectory], [p[0] for p in trajectory],
                color=C["mpc"], lw=2.5, solid_capstyle='round',
                zorder=4, label="MPC trajectory")
    rp = trajectory[-1]
    ax.add_patch(plt.Circle((rp[1], rp[0]), 0.55, color=C["robot"], zorder=6))
    ax.add_patch(plt.Circle((obs.c, obs.r), 0.8, color=C["dyn_obs"], zorder=5))
    ax.add_patch(plt.Circle((obs.c, obs.r), R_SAFE, color=C["dyn_obs"],
                 fill=False, linestyle='--', lw=0.8, zorder=5,
                 label=f"Safety radius R={R_SAFE}"))
    _draw_base(ax, gmap)
    ax.set_title(
        f"Layer 2 — MPC local replanner  |  H={H}  |  "
        f"obstacle appears at t={DYN_APPEAR}",
        fontsize=10, pad=6, color="#2c2c2a")
    ax.legend(fontsize=8, loc='upper left')
    plt.tight_layout()
    plt.savefig("./02_mpc_execution.png", dpi=150, bbox_inches='tight')
    plt.close()
    print("Saved: 02_mpc_execution.png")


def save_comparison(gmap, astar_path, trajectory, obs) -> None:
    fig, axes = plt.subplots(1, 2, figsize=(16, 6))
    fig.patch.set_facecolor(C["bg"])
    fig.suptitle("A* only vs A* + MPC — effect of dynamic obstacle",
                 fontsize=11, color="#2c2c2a")
    ax = axes[0]
    _draw_base(ax, gmap)
    if astar_path:
        ax.plot([p[1] for p in astar_path], [p[0] for p in astar_path],
                color=C["global"], lw=2.5, solid_capstyle='round', zorder=3)
    ax.add_patch(plt.Circle((obs.c, obs.r), 0.8, color=C["dyn_obs"], zorder=4))
    ax.add_patch(plt.Circle((obs.c, obs.r), R_SAFE, color=C["dyn_obs"],
                 fill=False, linestyle='--', lw=0.8, zorder=4))
    _draw_base(ax, gmap)
    ax.set_title("A* only — static plan, no replanning",
                 fontsize=10, pad=5, color="#2c2c2a")
    ax = axes[1]
    _draw_base(ax, gmap)
    if astar_path:
        ax.plot([p[1] for p in astar_path], [p[0] for p in astar_path],
                color=C["global"], lw=1.5, linestyle='--',
                zorder=2, alpha=0.5, label="A* reference")
    if len(trajectory) > 1:
        ax.plot([p[1] for p in trajectory], [p[0] for p in trajectory],
                color=C["mpc"], lw=2.5, solid_capstyle='round',
                zorder=3, label="MPC trajectory")
    ax.add_patch(plt.Circle((obs.c, obs.r), 0.8, color=C["dyn_obs"], zorder=4))
    ax.add_patch(plt.Circle((obs.c, obs.r), R_SAFE, color=C["dyn_obs"],
                 fill=False, linestyle='--', lw=0.8, zorder=4))
    _draw_base(ax, gmap)
    ax.set_title("A* + MPC — global plan + local replanning",
                 fontsize=10, pad=5, color="#2c2c2a")
    ax.legend(fontsize=9, loc='upper left')
    plt.tight_layout()
    plt.savefig("./03_comparison.png", dpi=150, bbox_inches='tight')
    plt.close()
    print("Saved: 03_comparison.png")


# ══════════════════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════════════════

def main() -> None:
    print("A* + MPC Path Planning")
    print("=" * 50)

    gmap = GridMap.default()

    # Layer 1: A* global path
    print("\nLayer 1 — A* global planner...")
    t0 = time.perf_counter()
    path_astar, explored = run_astar(gmap)
    print(f"  Explored {len(explored)} cells  |  path {len(path_astar)} cells  "
          f"|  {(time.perf_counter()-t0)*1000:.1f} ms")
    save_astar(gmap, explored, path_astar)

    # Layer 2: MPC execution
    print("\nLayer 2 — MPC local replanner...")
    print(f"  H={H}  W_track={W_TRACK}  W_smooth={W_SMOOTH}  W_obs={W_OBS}")
    print(f"  Obstacle appears at t={DYN_APPEAR} at {DYN_START}")

    obs = DynObs(r=float(DYN_START[0]), c=float(DYN_START[1]), vc=DYN_SPEED)
    t0  = time.perf_counter()
    trajectory, horizons = run_mpc(gmap, path_astar, obs)
    elapsed = (time.perf_counter() - t0) * 1000

    reached = math.hypot(
        trajectory[-1][0]-GOAL[0], trajectory[-1][1]-GOAL[1]) < 1.5
    print(f"  Steps: {len(trajectory)}  |  Goal reached: {'Yes' if reached else 'No'}")
    print(f"  MPC time: {elapsed:.0f} ms  ({elapsed/len(trajectory):.1f} ms/step)")

    save_mpc(gmap, path_astar, trajectory, horizons, obs)
    save_comparison(gmap, path_astar, trajectory, obs)

    print("\nDone. Check 01_astar.png, 02_mpc_execution.png, 03_comparison.png")


if __name__ == "__main__":
    main()

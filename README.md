## Overview
### 1. Layer 1: Global Planner (A*)
* **Method**: Discrete Graph Search on a 2D Grid.
* **Logic**: Uses an admissible **Manhattan-distance heuristic** ($f = g + h$) to ensure the shortest path is found.
*  **Role**: Provides a high-level reference trajectory that accounts for static environmental geometry.

### 2. Layer 2: Local Replanner (MPC)
* **Method**: Model Predictive Control using the **Receding Horizon** principle.
* **Logic**: Solves a constrained optimization problem at each time step to minimize a multi-objective cost function ($J$).
* **Objectives**:
  1. Path Tracking: Minimize cross-track error relative to the A* reference.
  2. Actuation Smoothness: Penalize high-frequency control inputs ($\Delta u$) to ensure feasible robot dynamics.
  3. Dynamic Safety: Maintain a safety buffer ($R_{safe}$) around moving obstacles.
  4. Static Obstacle Avoidance: Apply a heavy penalty ($W_{wall}$) to any trajectory point that intersects with a non-free grid cell, ensuring the robot respects the environment's physical boundaries.

### Open issue

After doubling the grid resolution (from $18 \times 28$ to $36 \times 56$), the MPC controller exhibited "short-sighted" behavior and began penetrating static obstacles (walls). While the A* global planner successfully handled the higher resolution, the local MPC tracking failed to maintain physical feasibility.

### Potential root cause

Cost Plateauing: At higher resolutions, a flat wall penalty creates a "cost well" with no gradient. If the optimizer enters a wall, it sees a uniform cost in every direction within its short horizon, leading to a local minimum where the robot remains trapped in the obstacle.

---
## Files
* **`planner.py`**: Handles environment discretization and the A* search algorithm.
* **`controller.py`**: Implements the continuous state-space MPC and dynamic obstacle physics.
* **`visualizer.py`**: A dedicated rendering module for generating simulation plots.
* **`main.py`**: The entry point orchestrating the hierarchical data flow.

## Results
A* search
![A* seach path](figure/01_astar.png?v=2)

local replanner with MPC
![Dynamic MPC Performance Plot](figure/02_mpc_execution.png?v=2)

---

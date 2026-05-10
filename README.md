## Overview
### 1. Layer 1: Global Planner (A*)
* **Method**: Discrete Graph Search on a 2D Grid.
* **Logic**: Uses an admissible **Manhattan-distance heuristic** ($f = g + h$) to ensure the shortest path is found.


### 2. Layer 2: Local Replanner (MPC)
* **Method**: Model Predictive Control using the **Receding Horizon** principle.
* **Logic**: Minimizes a multi-objective cost function ($J$) that balances path tracking, movement smoothness, and obstacle proximity.

## Files
* **`planner.py`**: Handles environment discretization and the A* search algorithm.
* **`controller.py`**: Implements the continuous state-space MPC and dynamic obstacle physics.
* **`visualizer.py`**: A dedicated rendering module for generating simulation plots.
* **`main.py`**: The entry point orchestrating the hierarchical data flow.

## Results
A* search
<img width="1200" height="600" alt="01_astar" src="https://github.com/user-attachments/assets/8155871c-e804-47f3-9bf1-712a21e98e41" />

local replanner with MPC
<img width="1200" height="600" alt="02_mpc_execution" src="https://github.com/user-attachments/assets/ee551ea2-a41d-4793-ba01-ca9a19f19878" />

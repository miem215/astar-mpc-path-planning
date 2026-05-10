## Overview
### 1. Layer 1: Global Planner (A*)
* **Method**: Discrete Graph Search on a 2D Grid.
* **Logic**: Uses an admissible **Manhattan-distance heuristic** ($f = g + h$) to ensure the shortest path is found.
* **Role**: Provides the high-level "mission intent" for the robot.

### 2. Layer 2: Local Replanner (MPC)
* **Method**: Model Predictive Control using the **Receding Horizon** principle.
* **Logic**: Minimizes a multi-objective cost function ($J$) that balances path tracking, movement smoothness, and obstacle proximity.
* **Role**: Reacts to dynamic obstacles (which appear mid-mission) that were unknown to the global planner.

## Modular Architecture
* **`planner.py`**: Handles environment discretization and the A* search algorithm.
* **`controller.py`**: Implements the continuous state-space MPC and dynamic obstacle physics.
* **`visualizer.py`**: A dedicated rendering module for generating simulation plots.
* **`main.py`**: The entry point orchestrating the hierarchical data flow.

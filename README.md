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
---
## To do's
* currently MPC use point-mass model for the robot, we could add more dimensions to the model
---
## Technical detail
### Model Predictive Control (MPC) Formulation

The local controller operates on a **Receding Horizon** principle. At each time step, it solves an optimization problem to find the best sequence of control inputs that minimizes a multi-objective cost function $J$.

### The Cost Function
The total cost $J$ is defined as the weighted sum of three primary objectives:

$$J = \sum_{k=1}^{H} \left( W_{track} \cdot J_{track} + W_{obs} \cdot J_{obs} + W_{smooth} \cdot J_{smooth} \right)$$

Where:
* **$H$ (Horizon):** The number of future steps the robot "looks ahead" to plan its immediate move.
* **$W$ (Weights):** Tunable parameters that determine the robot's "personality" (e.g., how much it values safety vs. speed).

### 1. Tracking Cost ($J_{track}$)
This term ensures the robot follows the global A* path as closely as possible. It calculates the squared Euclidean distance between the predicted state $x_k$ and the reference point $x_{ref,k}$:
$$J_{track} = \|x_k - x_{ref,k}\|^2$$

### 2. Obstacle Avoidance ($J_{obs}$)
To ensure safety, a repulsive penalty is applied when the robot enters the proximity of an obstacle. If the distance $d$ to an obstacle is less than the safety radius $R_{safe}$, the cost increases quadratically:
$$J_{obs} = \max(0, R_{safe} - d)^2$$

### 3. Smoothness Cost ($J_{smooth}$)
To prevent erratic or non-physical movements, we penalize large changes in the control input between steps:
$$J_{smooth} = \|u_k - u_{k-1}\|^2$$


### Controller Parameters
| Parameter | Value | Description |
| :--- | :--- | :--- |
| `Horizon (H)` | 5 | How many steps into the future the MPC predicts. |
| `dt` | 0.2s | Time step duration for the continuous simulation. |
| `W_TRACK` | 3.0 | Importance of staying on the A* path. |
| `W_OBS` | 10.0 | Importance of avoiding collisions (highest priority). |
| `W_smth` | 2.0 | Importance of avoiding jagged motions (lowest priority). |
| `R_SAFE` | 2.5 | Minimum distance (m) to maintain from obstacles. |


## Results
A* search
![A* seach path](figure/01_astar.png?v=2)

local replanner with MPC
![Dynamic MPC Performance Plot](figure/02_mpc_execution.png?v=2)

---

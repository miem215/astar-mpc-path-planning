from planner import GridMap, run_astar
from controller import MPCController, DynamicObstacle
from visualizer import save_plots  # <--- THE IMPORT YOU NEEDED
import numpy as np

def main():
    # 1. Setup
    gmap = GridMap.default_setup()
    start, goal = (9, 2), (6, 25)

    # 2. Global Plan (Discrete)
    astar_path, explored = run_astar(gmap, start, goal)

    # 3. Local Execution (Continuous)
    controller = MPCController()
    obs = DynamicObstacle(r=9.0, c=14.0)
    pos = np.array(start, dtype=float)
    trajectory = [pos.copy()]
    last_delta = np.zeros(2) # Initial value

    for t in range(100):
        if t == 6: obs.active = True
        obs.step(gmap)
        
        new_pos = controller.compute_step(pos, astar_path, obs, gmap, last_delta)
        trajectory.append(pos.copy())
        last_delta = new_pos - pos
        pos = new_pos
        if np.linalg.norm(pos - np.array(goal)) < 1.0:
            break

    # 4. Run the Visualizer
    print("Saving mission data to plots...")
    save_plots(gmap, explored, astar_path, trajectory, obs)

if __name__ == "__main__":
    main()
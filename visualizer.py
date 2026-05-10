import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

def save_plots(gmap, explored, astar_path, trajectory, obs):
    """Generates and saves the visual results of the mission."""
    # 01_astar.png
    fig1, ax1 = plt.subplots(figsize=(12, 6))
    _draw_base(ax1, gmap)
    for (r, c) in explored:
        ax1.add_patch(plt.Rectangle((c-.45, r-.45), .9, .9, color='#B5D4F4', alpha=0.6))
    ax1.plot([p[1] for p in astar_path], [p[0] for p in astar_path], color='#378ADD', lw=2.5)
    ax1.set_title("Layer 1: A* Global Plan")
    plt.savefig("./figure/01_astar.png")
    plt.close()

    # 02_mpc_execution.png
    fig2, ax2 = plt.subplots(figsize=(12, 6))
    _draw_base(ax2, gmap)
    ax2.plot([p[1] for p in astar_path], [p[0] for p in astar_path], 'k--', alpha=0.3, label="Reference")
    ax2.plot([p[1] for p in trajectory], [p[0] for p in trajectory], color='#EF9F27', lw=2.5, label="MPC Path")
    ax2.add_patch(plt.Circle((obs.c, obs.r), 0.8, color='#F0997B')) # Dynamic Obs
    ax2.add_patch(plt.Circle((obs.c, obs.r), 2.5, color='#F0997B', fill=False, ls='--')) # Safety Radius
    plt.legend()
    ax2.set_title("Layer 2: MPC Local Tracking")
    plt.savefig("./figure/02_mpc_execution.png")
    plt.close()

def _draw_base(ax, gmap):
    """Internal helper to draw the static environment."""
    ax.set_facecolor("#ffffff")
    for r in range(gmap.rows):
        for c in range(gmap.cols):
            if gmap.grid[r][c]:
                ax.add_patch(plt.Rectangle((c-.45, r-.45), .9, .9, color='#3d3d3a'))
    ax.set_xlim(-0.5, gmap.cols - 0.5)
    ax.set_ylim(-0.5, gmap.rows - 0.5)
    ax.invert_yaxis()
    ax.set_aspect('equal')
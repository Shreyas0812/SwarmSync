"""
Python script to visualize multi-agent trajectories from trajectories.txt
This is equivalent to the MATLAB plot_results.m script
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from mpl_toolkits.mplot3d import Axes3D

# Configuration flags
view_states = False
view_animation = True

# Read the data file - handle ragged arrays (different column counts per row)
with open('trajectories.txt', 'r') as f:
    lines = f.readlines()

# Parse the file manually since rows have different column counts
data_rows = []
max_cols = 0

for line in lines:
    values = [float(x) for x in line.strip().split()]
    data_rows.append(values)
    max_cols = max(max_cols, len(values))

# Create padded matrix (pad with zeros where needed)
M = np.zeros((len(data_rows), max_cols))
for i, row in enumerate(data_rows):
    M[i, :len(row)] = row

# Extract parameters from first line
N = int(M[0, 0])  # Number of agents
N_cmd = int(M[0, 1])  # Number of commanded agents
pmin = M[0, 2:5]  # Minimum position bounds [x, y, z]
pmax = M[0, 5:8]  # Maximum position bounds [x, y, z]

# Extract initial positions (lines 2-4, columns 1:N)
po = M[1:4, 0:N]  # Shape: (3, N)
po = po.T[np.newaxis, :, :]  # Reshape to (1, N, 3)

# Extract final/target positions (lines 5-7, columns 1:N_cmd)
pf = M[4:7, 0:N_cmd]  # Shape: (3, N_cmd)
pf = pf.T[np.newaxis, :, :]  # Reshape to (1, N_cmd, 3)

# Extract trajectory data (lines 8 onwards)
start = 7  # Python uses 0-based indexing, so line 8 is index 7
final = start + 3 * N_cmd
all_pos = M[start:final, :]

# Get actual number of time steps from the data
num_timesteps = len(data_rows[start])

# Organize trajectories by agent
# pk will have shape (N_cmd, time_steps, 3)
pk = np.zeros((N_cmd, num_timesteps, 3))

for i in range(N_cmd):
    # Extract x, y, z for agent i
    pk[i, :, 0] = M[start + 3*i, :num_timesteps]      # x coordinate
    pk[i, :, 1] = M[start + 3*i + 1, :num_timesteps]  # y coordinate
    pk[i, :, 2] = M[start + 3*i + 2, :num_timesteps]  # z coordinate

# Time array (assuming 0.01s time step)
T = 0.01 * (pk.shape[1] - 1)
t = np.arange(0, T + 0.01, 0.01)
if len(t) > pk.shape[1]:
    t = t[:pk.shape[1]]

# Plot states if requested
if view_states:
    # Figure 1: Distance to target over time
    fig1 = plt.figure(1, figsize=(10, 6))
    for i in range(N_cmd):
        # Calculate distance to target
        diff = pk[i, :, :] - pf[0, i, :]
        dist = np.sqrt(np.sum(diff**2, axis=1))
        plt.plot(t[:len(dist)], dist, linewidth=1.5, label=f'Agent {i+1}')
    plt.grid(True)
    plt.xlabel('t [s]')
    plt.ylabel('Distance to target [m]')
    plt.legend()
    plt.title('Distance to Target Over Time')

    # Figure 2: Individual position components
    fig2, axes = plt.subplots(3, 1, figsize=(10, 10))
    labels = ['x [m]', 'y [m]', 'z [m]']

    for i in range(N_cmd):
        for axis_idx in range(3):
            axes[axis_idx].plot(t[:pk.shape[1]], pk[i, :, axis_idx], 
                              linewidth=1.5, label=f'Agent {i+1}')
            axes[axis_idx].axhline(pmin[axis_idx], color='r', 
                                  linestyle='--', linewidth=1.5)
            axes[axis_idx].axhline(pmax[axis_idx], color='r', 
                                  linestyle='--', linewidth=1.5)
            axes[axis_idx].set_ylabel(labels[axis_idx])
            axes[axis_idx].set_xlabel('t [s]')
            axes[axis_idx].grid(True)
            axes[axis_idx].legend()

    plt.tight_layout()
    plt.show()

# Downsample for better visualization
pk_downsampled = pk[:, ::20, :]

# Animation of transition
if view_animation:
    fig = plt.figure(3, figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Generate distinct colors for each agent
    colors = plt.cm.tab10(np.linspace(0, 1, N))

    def init():
        ax.clear()
        ax.set_xlim(pmin[0], pmax[0])
        ax.set_ylim(pmin[1], pmax[1])
        ax.set_zlim(0, pmax[2])
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.grid(True)
        return []

    def animate(k):
        ax.clear()
        ax.set_xlim(pmin[0], pmax[0])
        ax.set_ylim(pmin[1], pmax[1])
        ax.set_zlim(0, pmax[2])
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.grid(True)
        ax.set_title(f'Time: {k*20*0.01:.2f}s')

        for i in range(N):
            if i < N_cmd:
                # Plot current position
                ax.plot([pk_downsampled[i, k, 0]], 
                       [pk_downsampled[i, k, 1]], 
                       [pk_downsampled[i, k, 2]], 
                       'o', markersize=10, color=colors[i], label=f'Agent {i+1}')

                # Plot initial position
                ax.plot([po[0, i, 0]], [po[0, i, 1]], [po[0, i, 2]], 
                       '^', markersize=10, color=colors[i])

                # Plot target position
                ax.plot([pf[0, i, 0]], [pf[0, i, 1]], [pf[0, i, 2]], 
                       'x', markersize=10, color=colors[i], linewidth=3)
            else:
                # Only plot initial position for agents without commands
                ax.plot([po[0, i, 0]], [po[0, i, 1]], [po[0, i, 2]], 
                       '^', markersize=10, color=colors[i])

        ax.legend()
        return []

    anim = animation.FuncAnimation(fig, animate, init_func=init,
                                 frames=pk_downsampled.shape[1], 
                                 interval=50, blit=False, repeat=True)

    plt.show()

print("Visualization complete!")
print(f"Number of agents: {N}")
print(f"Number of commanded agents: {N_cmd}")
print(f"Number of time steps: {pk.shape[1]}")
print(f"Total time: {T:.2f}s")

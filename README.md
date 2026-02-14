# SwarmSync 

**Distributed Model Predictive Control (DMPC) for Multi-Drone Motion Planning with Dynamic Goal Locations**

SwarmSync extends traditional DMPC approaches to handle dynamic goal tracking in multi-agent drone systems using physics-based simulation with `gym-pybullet-drones`.

---

## Overview

Traditional DMPC approaches assume static goal locations known at the start. **SwarmSync** addresses real-world scenarios where drones need to:
- Track moving targets in real-time
- Adapt trajectories to changing mission objectives
- Maintain collision-free paths with dynamic goals
- Coordinate multiple agents in dense environments

---

## Technical Approach

### Core Components

- **Bézier Curve Parameterization**: Smooth trajectory generation
- **On-Demand Collision Avoidance**: 50% faster than traditional BVC methods
- **Event-Triggered Replanning**: Adaptive response to disturbances and goal changes
- **Distributed QP Optimization**: Real-time solving at 5 Hz planning rate
- **gym-pybullet-drones Integration**: Physics-based quadrotor dynamics with Crazyflie 2.0 models

### Key Innovation

Adapting the optimization problem to track moving targets while preserving collision-free guarantees in a realistic physics simulator.

---

## Features

- ✅ Multi-agent trajectory planning
- ✅ Dynamic goal location tracking
- ✅ Real-time collision avoidance
- ✅ Physics-based simulation environment
- ✅ 3D trajectory visualization with PyBullet
- ✅ Performance analysis and plotting tools

---

## Installation

### Prerequisites

```bash
# Python 3.7+
pip install numpy matplotlib pybullet pybullet_data tqdm
```

### Clone the Repository

```bash
git clone https://github.com/Shreyas0812/SwarmSync.git
cd SwarmSync
```

---

## 🎮 Usage

### Visualize Trajectories with Static Goals

```bash
python viz_trajectory.py
```

### Visualize Trajectories with Dynamic Goals

```bash
python viz_trajectory_goals.py --trajectory trajectories.txt --goals goals.txt
```

Or use the shorthand:

```bash
python viz_trajectory_goals.py -t trajectories.txt -g goals.txt
```

### Plot Results

```bash
python plot_results_python.py
```

This generates:
- Distance to target over time
- 3D trajectory animations
- Performance metrics

---

## Data Format

### `trajectories.txt`

Contains the planned trajectories for all drones:
- Line 1: `N N_cmd pmin[3] pmax[3]` (metadata)
- Lines 2-4: Initial positions `po` for N drones (x, y, z)
- Lines 5-7: Final positions `pf` for N_cmd commanded drones
- Lines 8+: Trajectory data `pk` (3 lines per drone: x, y, z coordinates over time)

### `goals.txt`

Contains dynamic goal trajectories:
- 3 lines per drone (x, y, z coordinates)
- Each line has timestep columns showing goal evolution

---

## Project Structure

```
SwarmSync/
│
├── viz_trajectory_goals.py    # PyBullet visualization with dynamic goals
├── viz_trajectory.py           # PyBullet visualization with static goals
├── plot_results_python.py      # MATLAB-equivalent plotting script
│
├── trajectories.txt            # Generated trajectory data
├── goals.txt                   # Dynamic goal trajectories
│
├── DMPC-Pitch.md               # Project presentation (Marp slides)
├── DMPC-Pitch.pdf              # Compiled presentation
│
└── README.md                   # This file
```


### Expected Performance (from base paper)

- ✓ 90%+ success rate with 30 agents in dense environments
- ✓ 50% reduction in travel time vs. traditional methods
- ✓ Real-time capable (20 Hz control with 20 drones)

---

## Contributors

- **Shreyas Raorane** ([@Shreyas0812](https://github.com/Shreyas0812))
- **Kabir Puri** ([@kaRpuri](https://github.com/kaRpuri))


---


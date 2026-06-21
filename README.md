# SwarmSync

**Distributed Model Predictive Control (DMPC) for Multi-Drone Motion Planning with Dynamic Goal Locations and Online Task Reallocation**

SwarmSync extends the online DMPC framework of Luis et al. (2020) in two directions: (1) goals can move during execution (circular, translating, or random-jump motion), and (2) an online Hungarian-algorithm task reallocation layer periodically re-assigns drones to goals to minimize total travel distance.

---

## What This Repo Does

Traditional DMPC fixes the robot-to-goal assignment at the start and never revisits it. SwarmSync adds:

- **Dynamic goals** — goals move during flight (circular orbits, linear translation, random jumps)
- **Online task reallocation** — every `reallocation_period` seconds, the Hungarian algorithm re-solves the assignment problem and updates drone goals if a better assignment is found
- **9 curated test scenarios** — from a 4-drone diagonal swap to denser multi-agent formations, plus BVC vs. on-demand collision-avoidance comparisons
- **Scalability experiments** — automated sweep up to 64 agents
- **PyBullet visualization** — replay planned trajectories in a 3D GUI with Crazyflie drone models

The C++ solver lives in the `online_dmpc` submodule ([Shreyas0812/online_dmpc](https://github.com/Shreyas0812/online_dmpc)).

---

## Documentation

| Document | What's in it |
|---|---|
| [`docs/THEORY.md`](docs/THEORY.md) | The math — dynamics model, Bézier parameterization, the DMPC QP, collision avoidance (on-demand & BVC), Hungarian reallocation |
| [`docs/Report.md`](docs/Report.md) | Full project writeup with methodology, results, and discussion |
| [`docs/Visualisation_guide.md`](docs/Visualisation_guide.md) | How to read each generated figure |
| [`online_dmpc/README.md`](online_dmpc/README.md) | Building and running the C++ solver directly |

---

## Results

All experiments ran 3 independent trials per scenario. Key findings:

| Scenario | Method | Solving Freq (Hz) | Success Rate | Collision Rate |
|---|---|---|---|---|
| Scenario 1 (4 agents, diagonal swap) | Static | 406.6 ± 10.2 | 100% | 0% |
| Scenario 1 | With Reallocation | 400.4 ± 6.4 | 100% | 0% |
| Scenario 2 (4 agents, dense cross) | Static | 321.5 ± 21.9 | 100% | 0% |
| Scenario 2 | With Reallocation | 313.1 ± 4.0 | 100% | 0% |
| Scenario 3 (4 agents, circle rotate) | Static | 235.7 ± 5.8 | 100% | 0% |
| Scenario 3 | With Reallocation | 261.7 ± 12.5 | 100% | 0% |

- **Zero collisions** across all scenarios and methods
- **100% goal success rate** across all scenarios and methods
- Reallocation overhead is minimal (~2% drop in solving frequency)
- Scenario 2 required zero reallocations — static assignment was already optimal, showing the system correctly identifies when reallocation isn't needed
- Solver runs comfortably above 200 Hz in all configurations (real-time capable)

See [`docs/Report.md`](docs/Report.md) for the full results table and analysis.

---

## Project Structure

```
SwarmSync/
├── README.md
│
├── docs/                              # Documentation & deliverables
│   ├── THEORY.md                      #   Math: dynamics, Bézier, DMPC QP, collision avoidance, Hungarian
│   ├── Report.md                      #   Full project writeup with results
│   ├── Visualisation_guide.md         #   Guide to the generated figures
│   ├── pitch/                         #   Pitch slides (Marp source + PDF)
│   ├── references/                    #   Luis et al. 2020 paper
│   └── submissions/                   #   Final course report & poster
│
├── experiments/                       # Experiment orchestration & analysis
│   ├── run_comprehensive_experiments.sh   #   All 9 scenarios (static vs reallocation)
│   ├── run_scalability_experiments.sh     #   Scalability sweep (4–64 agents)
│   ├── extract_metrics.py                 #   Aggregate experiment results
│   ├── visualize_results.py               #   Comparison figures
│   └── analyze_scalability.py             #   Scalability analysis & plots
│
├── viz/                               # Trajectory visualization
│   ├── viz_trajectory.py              #   PyBullet 3D replay — static goals
│   ├── viz_trajectory_goals.py        #   PyBullet 3D replay — dynamic goals
│   └── plot_results_python.py         #   Matplotlib trajectory plotter / animator
│
└── online_dmpc/                       # Git submodule — C++ DMPC solver (fork of Luis et al.)
    ├── cpp/
    │   ├── src/                       #   C++ source (simulator, generator, task_reallocation, bvc_avoidance)
    │   ├── include/                   #   Headers
    │   ├── config/                    #   config.json + scenario_1…9.json + help.txt
    │   └── results/                   #   Solver output (trajectories.txt, goals.txt)
    ├── matlab/                        #   Original paper's MATLAB implementation + BVC reference
    └── extras/                        #   Third-party MATLAB plotting utilities
```

All Python/shell tooling resolves paths relative to its own location, so scripts can be run from the repo root regardless of where they live.

---

## Quick Start

### 1. Clone

```bash
git clone --recurse-submodules https://github.com/Shreyas0812/SwarmSync.git
cd SwarmSync
```

> Already cloned without submodules? Run `git submodule update --init --recursive`.

### 2. Build the C++ solver

**Dependencies:** CMake ≥ 3.0, C++14 compiler, Eigen3. qpOASES and the Hungarian solver are bundled as submodules.

```bash
cd online_dmpc/cpp
mkdir build && cd build
cmake .. && make -j4
```

### 3. Run a scenario

```bash
# from online_dmpc/cpp/build/
./bin/run ../config/scenario_1.json     # a specific scenario
./bin/run                               # the shipped default
```

The shipped default (`config.json`) is an **8-drone antipodal circle swap** — a clean collision-avoidance demo. Output is written to `online_dmpc/cpp/results/trajectories.txt` and `goals.txt`.

### 4. Visualize

> Run visualization commands **from the repo root**. Install deps once: `pip install numpy matplotlib pybullet`.

**PyBullet 3D replay (static goals):**

```bash
python viz/viz_trajectory.py
```

> **Windows:** pybullet requires C++ Build Tools. Use `conda install -c conda-forge pybullet` if you don't have them.
>
> **Crazyflie URDF** (optional, falls back to spheres if missing):
> ```bash
> pip install git+https://github.com/utiasDSL/gym-pybullet-drones.git
> ```

Key parameters at the bottom of `viz/viz_trajectory.py`:

| Parameter | Default | Description |
|---|---|---|
| `gui` | `True` | Open PyBullet window; `False` for headless |
| `downsample` | `20` | Replay speed multiplier |
| `use_urdf` | `True` | Use Crazyflie model; falls back to spheres |

**PyBullet 3D replay (dynamic goals):**

```bash
python viz/viz_trajectory_goals.py --trajectory online_dmpc/cpp/results/trajectories.txt \
                                    --goals online_dmpc/cpp/results/goals.txt
```

**Matplotlib plotter:**

```bash
python viz/plot_results_python.py
```

Generates 3D trajectory animations and distance-to-target plots.

---

## Scenarios

Nine pre-configured scenarios live in `online_dmpc/cpp/config/`. Switch by passing the config file to the binary:

```bash
./bin/run ../config/scenario_3.json
```

### Reallocation vs Static Assignment

| Scenario | Agents | Description |
|---|---|---|
| `scenario_1.json` | 4 | Cross pattern diagonal swap — clearest reallocation benefit |
| `scenario_2.json` | 4 | Dense cross — static already optimal, reallocation is a no-op |
| `scenario_3.json` | 4 | Circle formation 90° rotation — maximum path crossing |

### BVC vs On-Demand Collision Avoidance

| Scenario | Agents | Description |
|---|---|---|
| `scenario_4.json` | 4 | Cross pattern with BVC (conservative, wider paths) |
| `scenario_5.json` | 4 | Cross pattern with on-demand (tighter coordination) |
| `scenario_6.json` | 6 | Dense 6-agent pattern stress-testing both methods |

### Dynamic / Moving Goals

| Scenario | Agents | Description |
|---|---|---|
| `scenario_7.json` | 4 | Translating goals — constant-velocity linear motion |
| `scenario_8.json` | 4 | Circular goals — goals orbit fixed centers |
| `scenario_9.json` | 4 | Combined — goals translate and rotate simultaneously |

> **Note on BVC:** the `bvc_avoidance.cpp` method is a *proactive* variant of the on-demand
> linearized collision constraint, not a literal Voronoi-cell partition — see `docs/THEORY.md` §4
> for the code-vs-theory distinction.

---

## Running Experiments

```bash
# from SwarmSync/ root

# Run all 9 scenarios (static vs reallocation comparison)
bash experiments/run_comprehensive_experiments.sh

# Run scalability sweep (4 → 64 agents)
bash experiments/run_scalability_experiments.sh

# Analyze and visualize results
python experiments/extract_metrics.py
python experiments/visualize_results.py
python experiments/analyze_scalability.py
```

Results and figures are saved to `online_dmpc/cpp/results/experiments/`.

> The `run_*.sh` scripts currently use Windows (CRLF) line endings; on Linux/macOS run them
> via `bash experiments/<script>.sh` after converting to LF (`dos2unix`) if your shell rejects them.

---

## Key Configuration Parameters

Edit `online_dmpc/cpp/config/config.json` (or any scenario JSON):

```json
{
  "N": 8,                          // Number of agents (default demo: 8-drone antipodal swap)
  "reallocation_enabled": false,   // Enable Hungarian reallocation (on in scenario_1 / scenario_3)
  "reallocation_period": 2.0,      // Seconds between reallocation checks
  "motion_type": "static",         // static | circular | translating | circular_translating | random_jump
  "collision_method": "on-demand", // on-demand | BVC
  "simulation_duration": 30        // Seconds
}
```

See `online_dmpc/cpp/config/help.txt` for a full parameter reference.

---

## Reference

C. E. Luis, M. Vukosavljev, and A. P. Schoellig, "Online Trajectory Generation with Distributed Model Predictive Control for Multi-Robot Motion Planning," *IEEE Robotics and Automation Letters*, vol. 5, no. 2, pp. 604–611, Apr. 2020.

---

## Contributors

- **Shreyas Raorane** ([@Shreyas0812](https://github.com/Shreyas0812))
- **Kabir Puri** ([@kaRpuri](https://github.com/kaRpuri))

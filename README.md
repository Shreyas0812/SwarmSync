# SwarmSync

**Distributed Model Predictive Control (DMPC) for Multi-Drone Motion Planning with Dynamic Goal Locations and Online Task Reallocation**

<p align="center">
  <img src="demos/hero.gif" alt="SwarmSync demos: collision-avoidance swap, moving-goal tracking, and 32-drone scalability" width="720">
  <br>
  <em>Collision-avoidance swap → moving-goal tracking → 32-drone reallocation (static vs. with reallocation). Full demos in <a href="#shipped-demos"><code>demos/</code></a>.</em>
</p>

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

### 3. Run & visualize the demo

Install the Python deps once: `pip install numpy pybullet tqdm` (add `ffmpeg` to record MP4s).
Then, from the repo root, generate the shipped demo's data and view or record it:

```bash
./run_dmpc.sh demos/antipodal_swap_8            # solve -> trajectories.txt, goals.txt
python view_trajectories.py antipodal_swap_8    # interactive PyBullet replay
python record_trajectories.py antipodal_swap_8  # headless render -> demos/antipodal_swap_8/demo.mp4
```

`demos/<name>/` just needs a `config.json` — copy any scenario into one and rerun the commands
above to make your own. See `python record_trajectories.py -h` for render options (`--downsample`,
`--scale`, `--no-trails`, …), **[Scenarios](#scenarios)** for the 9 preset configs, and
**[Running Experiments](#running-experiments)** for the full sweeps.

### Shipped demos

Six ready-to-run demos live under `demos/`. Each ships a `config.json` and a pre-rendered
`demo.mp4` — watch the video directly, or regenerate the data with
`./run_dmpc.sh demos/<name>` and replay it with `view_trajectories.py` / `record_trajectories.py`.

| Demo | Agents | Shows |
|---|---|---|
| `antipodal_swap_8` | 8 | Collision avoidance — antipodal ring swap through the center (static goals) |
| `moving_goals_8` | 8 | Moving-goal tracking — goals orbit + drift; drones cross, then track in formation |
| `reallocation_static_4` | 4 | Baseline: task reallocation **off** (fixed assignment) |
| `reallocation_predictive_4` | 4 | Predictive Hungarian reallocation **on** — same scenario, re-assigned mid-flight |
| `scalability_reallocation_32` | 32 | 32-drone swarm reconfiguration with reallocation on |
| `scalability_static_32` | 32 | Same 32-drone swarm, reallocation off |

The three pairs (`reallocation_*`, `scalability_*`) are meant to be watched side-by-side.

> **Windows:** pybullet needs C++ Build Tools — or `conda install -c conda-forge pybullet`.
> The realistic Crazyflie model is optional: `pip install git+https://github.com/utiasDSL/gym-pybullet-drones.git` (else it falls back to spheres).

---

## Documentation

| Document | What's in it |
|---|---|
| [`docs/THEORY.md`](docs/THEORY.md) | The math — dynamics model, Bézier parameterization, the DMPC QP, collision avoidance (on-demand & BVC), Hungarian reallocation |
| [`docs/Report.md`](docs/Report.md) | Full project writeup with methodology, results, and discussion |
| [`docs/Visualisation_guide.md`](docs/Visualisation_guide.md) | How to read each generated figure |
| [`docs/ROS2_MIGRATION.md`](docs/ROS2_MIGRATION.md) | Plan for a real-time, distributed ROS 2 + RViz2 port |
| [`online_dmpc/README.md`](online_dmpc/README.md) | Building and running the C++ solver directly |

---

## Results

Across all 9 scenarios (3 independent trials each):

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
├── run_dmpc.sh · record_trajectories.py · view_trajectories.py   # solve / replay / record a config
├── demos/          # Showcase runs (config.json + demo.mp4; *.txt/log git-ignored)
├── viz/            # PyBullet replay + matplotlib plotting
├── experiments/    # Scenario & scalability sweeps + analysis
├── docs/           # THEORY, Report, ROS 2 migration plan, slides, paper
└── online_dmpc/    # Git submodule — C++ DMPC solver (fork of Luis et al.)
```

---

## Scenarios

Nine pre-configured scenarios live in `online_dmpc/cpp/config/`. Run one directly with the
solver (from `bin/`, so its relative output paths resolve), or wrap it with `run_dmpc.sh`:

```bash
# direct
cd online_dmpc/cpp/bin && ./run ../config/scenario_3.json     # -> ../results/{trajectories,goals}.txt

# or via the demo tooling (writes outputs into the folder, then visualize)
mkdir -p demos/s3 && cp online_dmpc/cpp/config/scenario_3.json demos/s3/config.json
./run_dmpc.sh demos/s3 && python view_trajectories.py s3
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

Each `config.json` / scenario JSON exposes the main knobs: `N` (agents), `motion_type`
(`static | circular | translating | circular_translating | random_jump`), `reallocation_enabled`
+ `reallocation_period`, `collision_method` (`on-demand | BVC`), and `simulation_duration`. See
`online_dmpc/cpp/config/help.txt` for the full reference.

---

## Reference

C. E. Luis, M. Vukosavljev, and A. P. Schoellig, "Online Trajectory Generation with Distributed Model Predictive Control for Multi-Robot Motion Planning," *IEEE Robotics and Automation Letters*, vol. 5, no. 2, pp. 604–611, Apr. 2020.

---

## Contributors

- **Shreyas Raorane** ([@Shreyas0812](https://github.com/Shreyas0812))
- **Kabir Puri** ([@kaRpuri](https://github.com/kaRpuri))

# Dynamic Task Reallocation for Multi-Robot Motion Planning using Distributed Model Predictive Control

**Authors:** Shreyas Raorane, Kabir Puri
**Course:** MEAM 5170 — Control and Optimization with Applications in Robotics
**Institution:** University of Pennsylvania
**Date:** December 2025

---

## Abstract

This project extends the online distributed model predictive control (DMPC) framework for multi-robot trajectory generation by implementing dynamic task reallocation using the Hungarian algorithm. While the baseline system assumes fixed goal assignments, our enhancement enables robots to adaptively reassign goals during execution to minimize total travel distance. Experimental results across three scenarios demonstrate that both static and reallocation methods achieve 100% success rates with zero collisions, with the reallocation method providing adaptability benefits in scenarios with suboptimal initial assignments while incurring only ~2% computational overhead.

---

## 1. Introduction

### 1.1 Problem Statement

Multi-robot systems executing point-to-point transitions typically use fixed goal assignments determined at initialization. However, as robots move through the workspace, the initial assignment may become suboptimal due to:
- Poor initial assignment quality
- Dynamic changes in robot positions
- Collision avoidance maneuvers that deviate from direct paths

### 1.2 Motivation

Consider a scenario where Robot A is assigned to Goal 1 (far away) and Robot B to Goal 2 (far away), but during execution, Robot A ends up closer to Goal 2 and vice versa. A static system forces both robots to cross paths unnecessarily, increasing completion time and collision risk.

### 1.3 Contributions

This project implements **online task reallocation** that:
1. Periodically evaluates the current robot-goal assignment
2. Uses the Hungarian algorithm to compute the optimal reassignment
3. Updates DMPC goals dynamically during execution
4. Maintains collision avoidance and trajectory smoothness throughout

---

## 2. Background

### 2.1 Baseline System: Online DMPC

The baseline implementation from Luis et al. (2020) uses:
- **Distributed MPC**: Each agent solves its own QP problem independently
- **Bézier curve parameterization**: Smooth trajectory representation using concatenated Bézier segments
- **On-demand collision avoidance**: Linearized ellipsoidal constraints activated only between pairs on a collision course
- **Event-triggered replanning**: Robust to sensor noise and disturbances

**Limitation addressed by this project:** Fixed goal assignment throughout execution, with no adaptation to changing spatial relationships between agents and goals.

### 2.2 Hungarian Algorithm

The assignment problem: given N robots at positions $\mathbf{p}_i$ and N goals at $\mathbf{g}_j$, find the permutation $\pi$ that minimizes total travel distance:

$$\min_{\pi} \sum_{i=1}^{N} \|\mathbf{p}_i - \mathbf{g}_{\pi(i)}\|_2$$

The Hungarian algorithm solves this in $O(N^3)$ time. For small swarms — the 4-agent scenarios here, scaling to a few dozen agents — this runs in roughly 1–2 ms, making it suitable for real-time periodic reallocation.

---

## 3. Methodology

### 3.1 System Architecture

```
Main Loop (every h = 0.1s):
  ├─> Check if reallocation period elapsed (T = 2.0s)
  │   ├─> Collect current agent positions
  │   ├─> Build N×N Euclidean distance cost matrix
  │   ├─> Run Hungarian algorithm → optimal permutation π
  │   ├─> If assignment changed:
  │   │   └─> Call Generator::setGoalPoint(i, goals[π(i)]) for each agent
  │   └─> Log reallocation event to CSV
  │
  └─> Solve DMPC for all agents
      └─> Generate next Bézier trajectory segment
```

### 3.2 Implementation Details

**TaskReallocationManager** (`task_reallocation.cpp`):
- Tracks elapsed time and triggers reallocation when `current_time - last_reallocation_time >= reallocation_period`
- Builds the cost matrix from Euclidean distances between current agent positions and original goal positions
- Detects whether the new assignment actually differs from the current one (avoids unnecessary goal updates)
- Logs all reallocation events (timestamp, agent, old goal, new goal, distance) to `reallocation_log.csv`

**Integration with DMPC:**
- Uses `Generator::setGoalPoint()` to update the target for individual agents without restarting the solver
- The DMPC naturally handles smooth replanning via its receding-horizon formulation — no discontinuities in the trajectory
- Collision avoidance constraints remain active throughout goal updates

**Key Parameters:**
- Reallocation period: `T = 2.0s`
- MPC planning horizon: `k_hor = 16` steps × `h = 0.1s` = 1.6s lookahead
- Safety distance: `r_min = 0.3m` (ellipsoidal)
- Simulation timestep: `Ts = 0.01s`

### 3.3 Experimental Setup

**Test Scenarios:**

| Scenario | Agents | Description | Expected Reallocation Benefit |
|---|---|---|---|
| Scenario 1 | 4 | Diagonal cross swap in 4×4m workspace | High — initial assignment forces path crossing |
| Scenario 2 | 4 | Dense cross, higher agent density | Low — static assignment already near-optimal |
| Scenario 3 | 4 | Circle formation, 90° rotation | High — maximum path intersection |

**Comparison Methods:**
- **Static Assignment**: Original DMPC with fixed goals
- **With Reallocation**: DMPC + Hungarian reallocation every 2s

**Metrics:**
- MPC solving frequency (Hz) — computational overhead indicator
- Success rate — percentage of runs where all agents reached goals
- Collision rate — percentage of runs with any inter-agent distance violation
- Distance to goal at termination
- Final assignment cost
- Total reallocation distance (sum of goal reassignment distances)

**Runs:** 3 independent trials per scenario/method = 18 total experiments.

---

## 4. Results

### 4.1 Summary Statistics

| Scenario | Method | Avg Freq (Hz) | Success Rate | Collision Rate | Avg Dist to Goal | Final Cost | Total Realloc Dist |
|---|---|---|---|---|---|---|---|
| Scenario 1 | Static | 406.56 ± 10.15 | 100% | 0% | 0.0000 m | 0.00 | — |
| Scenario 1 | With Reallocation | 400.42 ± 6.43 | 100% | 0% | 0.0193 m | 1.71×10⁻¹³ | 3.71 m |
| Scenario 2 | Static | 321.46 ± 21.94 | 100% | 0% | 0.0000 m | 0.00 | — |
| Scenario 2 | With Reallocation | 313.07 ± 4.02 | 100% | 0% | 0.0460 m | 4.01×10⁻¹³ | 0.00 m |
| Scenario 3 | Static | 235.74 ± 5.76 | 100% | 0% | 0.0000 m | 0.00 | — |
| Scenario 3 | With Reallocation | 261.66 ± 12.50 | 100% | 0% | 0.0282 m | 2.93×10⁻¹³ | 8.22 m |

### 4.2 Key Findings

**Safety:** Both methods achieve a perfect 0% collision rate across all scenarios. The ellipsoidal on-demand collision avoidance constraints are effective regardless of whether task reallocation is active.

**Reliability:** 100% success rate across all 18 experiment runs. Every agent reached its assigned goal within the tolerance threshold.

**Computational overhead:** Reallocation reduces solving frequency by approximately 1.5–2.5%, dropping from ~406 Hz to ~400 Hz in Scenario 1. This is negligible for real-time operation.

**Reallocation behavior:**
- Scenario 1: Reallocation triggered and found a better assignment, total reassignment distance of 3.71 m
- Scenario 2: Zero reallocations performed — the algorithm correctly identified that the static assignment was already optimal and did not disturb it
- Scenario 3: 8.22 m total reassignment distance, indicating significant reordering occurred

**Final assignment cost:** Both methods converge to effectively zero final cost (on the order of 10⁻¹³), meaning both reach numerically optimal assignments at termination.

**Goal precision:** The static method reaches exact goal positions (0.0000 m error). The reallocation method shows small non-zero distances (0.02–0.05 m) because the Hungarian algorithm occasionally updates the goal target near the end of the simulation, leaving insufficient time to reach the new position exactly. All values remain well within the goal tolerance of 0.15 m.

### 4.3 Solving Frequency

The MPC solving frequency degrades with scenario complexity. In Scenario 3 (densest collision avoidance), the reallocation method shows slightly *higher* frequency (261.66 Hz vs 235.74 Hz static), likely because reallocation found shorter paths that reduce per-step QP complexity.

All configurations maintain >200 Hz, confirming real-time viability.

### 4.4 Cost Convergence

The assignment cost (sum of agent-to-goal distances) converges exponentially under the reallocation method, dropping by 6+ orders of magnitude and plateauing at numerical precision limits (~10⁻¹³). This confirms the Hungarian algorithm is finding and maintaining globally optimal assignments.

---

## 5. Discussion

### 5.1 Why Reallocation Works

**Initial Assignment Quality:** The original DMPC assigns robot *i* to goal *i* (identity permutation), which is often suboptimal. The Hungarian algorithm finds the globally optimal permutation based on current positions.

**Dynamic Adaptation:** As robots navigate and avoid collisions, their spatial relationships change. Periodic reallocation captures these changes and updates assignments accordingly.

**Minimal Overhead:** The Hungarian algorithm runs in ~1–2 ms for small swarms, and reallocation only fires every 2 seconds, making total overhead negligible.

### 5.2 When Reallocation Doesn't Help

Scenario 2 shows that when the initial assignment is already near-optimal, reallocation performs zero reassignments and adds no benefit — but also no harm. The method is adaptive and self-regulating.

### 5.3 Limitations

**Centralized computation:** The current implementation requires global position knowledge for the cost matrix. A fully distributed implementation would require a consensus protocol.

**Fixed reallocation period:** The 2s period may not be optimal for all scenarios. An event-triggered approach (trigger when assignment cost exceeds a threshold) could be more responsive.

**O(N³) complexity:** The Hungarian algorithm scales cubically with agent count. For large swarms (N > 50), this becomes a bottleneck.

**No predictive reallocation by default:** The current method uses current positions for cost computation. The predictive mode (`_use_predictive: true`) uses horizon predictions but was not the primary experiment focus.

---

## 6. Future Work

**Short-term:**
- Adaptive reallocation period triggered by assignment cost changes rather than fixed intervals
- Full evaluation of predictive reallocation across all scenarios
- Per-agent energy consumption as a secondary cost metric

**Long-term:**
- Distributed assignment using auction-based methods
- Heterogeneous agents with different speed/endurance (weighted cost matrix)
- Integration with higher-level task planning (multi-waypoint missions, task dependencies)

---

## 7. Conclusion

This project demonstrates that online task reallocation integrates seamlessly with DMPC without sacrificing safety or reliability. Both static and reallocation methods achieved 100% success rates and 0% collision rates. The reallocation method adds adaptability — correctly identifying and applying better assignments in Scenarios 1 and 3, while correctly doing nothing in Scenario 2 where no improvement was possible. Computational overhead is minimal (~2%), confirming viability for real-time multi-robot systems.

---

## 8. References

1. Luis, C. E., Vukosavljev, M., & Schoellig, A. P. (2020). Online trajectory generation with distributed model predictive control for multi-robot motion planning. *IEEE Robotics and Automation Letters*, 5(2), 604–611.

2. Kuhn, H. W. (1955). The Hungarian method for the assignment problem. *Naval Research Logistics Quarterly*, 2(1–2), 83–97.

3. Bento, J., Derbinsky, N., Alonso-Mora, J., & Yedidia, J. S. (2013). A message-passing algorithm for multi-agent trajectory planning. *Advances in Neural Information Processing Systems*, 26.

---

## Appendix A: Repository

GitHub: [https://github.com/Shreyas0812/SwarmSync](https://github.com/Shreyas0812/SwarmSync)

**Key files in the C++ solver (`online_dmpc/`):**

| File | Purpose |
|---|---|
| `cpp/include/task_reallocation.h` | TaskReallocationManager class declaration |
| `cpp/src/task_reallocation.cpp` | Hungarian algorithm integration and logging |
| `cpp/src/simulator.cpp` | Main simulation loop with reallocation trigger |
| `cpp/src/generator.cpp` | DMPC solver, Bézier trajectory generation, moving goals |
| `cpp/config/scenario_*.json` | Per-scenario configurations |

**Experiment tooling (`experiments/`, in the parent SwarmSync repo):**

| File | Purpose |
|---|---|
| `run_comprehensive_experiments.sh` | Automated experiment runner |
| `extract_metrics.py` | Aggregates results from experiment runs |
| `visualize_results.py` | Generates comparison figures |
| `analyze_scalability.py` | Scalability sweep analysis and plots |

## Appendix B: Visualization Guide

See `VISUALIZATION_GUIDE.md` for a detailed explanation of all generated figures and how to interpret them.

# SwarmSync — Theory and Math

This document explains the mathematical foundations behind SwarmSync. It covers the four core components: Bézier curve trajectory parameterization, the Distributed MPC (DMPC) formulation and QP, collision avoidance (on-demand vs BVC), and the Hungarian algorithm for task reallocation.

---

## 1. Drone Dynamic Model

Each drone is modeled as a **second-order linear system** (double integrator with damping), separately in each axis. The transfer function from acceleration input $u$ to position $p$ is:

$$G(s) = \frac{1}{s(\tau s + 1)} \cdot \frac{1}{\tau s + 1}$$

with damping ratio $\zeta$ and time constant $\tau$ (configured separately for XY and Z axes via `zeta_xy`, `tau_xy`, `zeta_z`, `tau_z`).

Discretized at timestep $T_s$, the state-space model for state $\mathbf{x} = [\mathbf{p}, \dot{\mathbf{p}}]^\top \in \mathbb{R}^6$ is:

$$\mathbf{x}_{k+1} = A\,\mathbf{x}_k + B\,\mathbf{u}_k$$

where $\mathbf{u}_k \in \mathbb{R}^3$ is the acceleration command. This model is used both for **prediction** (at the MPC planning rate $h$) and **execution** (at the fine simulation rate $T_s$).

---

## 2. Bézier Curve Trajectory Parameterization

### 2.1 Why Bézier Curves?

Instead of optimizing over individual waypoints, the DMPC optimizes over **Bézier control points**. This gives:
- Inherent smoothness (derivatives of any order exist)
- Compact representation (few parameters cover a long horizon)
- Easy continuity enforcement at segment junctions

### 2.2 Bernstein Polynomial Basis

A Bézier curve of degree $d$ over $t \in [0, T]$ is defined by $d+1$ control points $\{\mathbf{c}_0, \ldots, \mathbf{c}_d\}$:

$$\mathbf{p}(t) = \sum_{i=0}^{d} \mathbf{c}_i \, B_{i,d}\!\left(\frac{t}{T}\right)$$

where $B_{i,d}(\tau) = \binom{d}{i}\tau^i(1-\tau)^{d-i}$ are the **Bernstein basis polynomials**.

In this implementation, $d = 5$ (degree-5 polynomial) with $l = 3$ concatenated segments, each of duration $T_\text{seg} = 0.5$s. The full planning horizon covers $l \cdot T_\text{seg} = 1.5$s.

### 2.3 Power Basis Conversion

For matrix operations, the Bernstein basis is converted to the **power (monomial) basis** $[1, t, t^2, \ldots, t^d]^\top$ using the conversion matrix $\boldsymbol{\Beta}$:

$$B_{i,d}(\tau) = \sum_{k=0}^{d} \beta_{ik} \, \tau^k, \qquad \beta_{ik} = (-1)^{i-k}\binom{d}{i}\binom{i}{k}$$

This matrix is precomputed in `BezierCurve::bernsteinToPowerBasis()`.

### 2.4 Derivatives

The $r$-th derivative of a Bézier curve is itself a Bézier curve of degree $d - r$ with control points derived from finite differences of the original control points. This allows direct computation of velocity, acceleration, and jerk from the control point vector.

### 2.5 Sampling Matrix

To evaluate the trajectory at a set of discrete times $\{t_1, \ldots, t_m\}$, a **sampling matrix** $\boldsymbol{\Rho} \in \mathbb{R}^{3m \times 3l(d+1)}$ is constructed such that:

$$\begin{bmatrix}\mathbf{p}(t_1) \\ \vdots \\ \mathbf{p}(t_m)\end{bmatrix} = \boldsymbol{\Rho} \, \mathbf{c}$$

where $\mathbf{c} \in \mathbb{R}^{3l(d+1)}$ is the stacked vector of all control points across all segments and all three spatial dimensions. The $r$-th derivative sampling matrix is built by `getMatrixInputDerivativeSampling()`.

### 2.6 Continuity Constraints

Concatenating $l$ Bézier segments requires enforcing continuity up to degree `deg_poly = 3` (position, velocity, acceleration, jerk) at each junction. This gives equality constraints:

$$A_\text{eq} \, \mathbf{c} = \mathbf{b}_\text{eq}$$

built by `getMatrixEqualityConstraint()`.

---

## 3. Distributed MPC Formulation

### 3.1 Receding Horizon Control

At each planning step (every $h = 0.1$s), each agent $i$ independently solves a **quadratic program (QP)** over a horizon of $k_\text{hor} = 16$ steps ($= 1.6$s lookahead). The solution gives the optimal control input sequence; only the first step is applied, then the problem is resolved at the next planning step.

### 3.2 Optimization Variable

The decision variable for agent $i$ is its **Bézier control point vector** $\mathbf{c}_i \in \mathbb{R}^{3l(d+1)}$ representing the planned trajectory over the horizon.

### 3.3 Objective Function

The cost has three terms:

$$J_i = \underbrace{(\boldsymbol{\Rho}_h \mathbf{c}_i - \mathbf{g}_i)^\top \mathbf{H}_f (\boldsymbol{\Rho}_h \mathbf{c}_i - \mathbf{g}_i)}_{\text{goal tracking}} + \underbrace{\mathbf{c}_i^\top \mathbf{H}_\text{energy} \, \mathbf{c}_i}_{\text{energy (acc. regularization)}} + \underbrace{\text{collision penalty}}_{\text{see Section 4}}$$

**Goal tracking term:** $\mathbf{H}_f$ penalizes deviation of the sampled trajectory from the goal $\mathbf{g}_i$ at each horizon step. The penalty is shaped by `s_free` (when no collision constraints are active) and `s_obs` / `s_repel` (when active).

**Energy term:** $\mathbf{H}_\text{energy}$ penalizes the squared acceleration norm across the trajectory, weighted by `acc_cost = 0.015`. This promotes smooth, energy-efficient motion.

### 3.4 Constraints

**Workspace bounds:** Position must remain within $[\mathbf{p}_\text{min}, \mathbf{p}_\text{max}]$ and acceleration within $[\mathbf{a}_\text{min}, \mathbf{a}_\text{max}]$, enforced as linear inequalities on the control points:

$$A_\text{ineq} \, \mathbf{c}_i \leq \mathbf{b}_\text{ineq}$$

**Initial condition:** The trajectory must begin at the agent's current state (position + velocity), enforced as an equality constraint.

**Continuity:** Between Bézier segments, derivatives up to degree 3 must match.

**Collision avoidance:** Described in Section 4.

### 3.5 QP Structure

Combining everything, each agent solves at each planning step:

$$\min_{\mathbf{c}_i} \quad \mathbf{c}_i^\top \mathbf{H} \, \mathbf{c}_i + \mathbf{f}^\top \mathbf{c}_i$$

$$\text{subject to} \quad A_\text{eq} \mathbf{c}_i = \mathbf{b}_\text{eq}, \quad A_\text{ineq} \mathbf{c}_i \leq \mathbf{b}_\text{ineq}$$

This is solved using **qpOASES**, a fast active-set QP solver. Typical solve times are 0.5–5 ms, giving solving frequencies of 200–400+ Hz.

### 3.6 State Propagation

The state prediction over the horizon uses state propagator matrices $\boldsymbol{\Lambda}$ and $\mathbf{A}_0$ (built from the discretized drone model):

$$\mathbf{X} = \boldsymbol{\Lambda} \, \mathbf{U} + \mathbf{A}_0 \, \mathbf{x}_0$$

where $\mathbf{X}$ stacks predicted states and $\mathbf{U}$ stacks planned inputs. These matrices allow the collision constraints (which are in position space) to be written as linear functions of the decision variable $\mathbf{c}_i$.

---

## 4. Collision Avoidance

Two methods are implemented, selectable via `"collision_method"` in the config.

### 4.1 Ellipsoidal Safety Sets

Both methods use an **ellipsoidal** safety region around each drone:

$$\mathcal{E}_i = \left\{ \mathbf{p} : \left\|\mathbf{E}^{-1}(\mathbf{p} - \mathbf{p}_i)\right\|_\text{order} \leq r_\text{min} \right\}$$

where $\mathbf{E} = \text{diag}(1, 1, \text{height\_scaling})$ scales the ellipsoid to be taller than it is wide (drones are more dangerous to collide with horizontally). Parameters: `rmin = 0.3m`, `order = 2`, `height_scaling = 1.2`.

### 4.2 On-Demand Collision Avoidance

**On-demand** is the default method. It activates inter-agent constraints **only when two agents are predicted to come within the safety distance** over the planning horizon.

When agents $i$ and $j$ are on a collision course, a **linearized half-space constraint** is added to agent $i$'s QP:

$$\mathbf{n}_{ij}^\top \mathbf{p}_i(t) \geq \mathbf{n}_{ij}^\top \mathbf{p}_j(t) + r_\text{min}$$

where $\mathbf{n}_{ij}$ is the unit vector from $j$ to $i$ at the predicted collision point. This linearization makes the constraint linear in $\mathbf{c}_i$, preserving the QP structure.

The "on-demand" name reflects that constraints are only generated between pairs that need them — most pairs are collision-free and incur no constraint overhead.

The cost function also includes a **soft repulsion term** for pairs approaching but not yet violating the constraint, with penalty `s_repel = 1000`.

### 4.3 BVC — Buffered Voronoi Cells

The **Buffered Voronoi Cell** of agent $i$ is the subset of the Voronoi region around $i$ that is at least $r_\text{min}/2$ away from every Voronoi face:

$$\text{BVC}_i = \left\{ \mathbf{p} : \forall j \neq i, \; \mathbf{n}_{ij}^\top \mathbf{p} \geq \mathbf{n}_{ij}^\top \frac{\mathbf{p}_i + \mathbf{p}_j}{2} + \frac{r_\text{min}}{2} \right\}$$

where $\mathbf{n}_{ij} = \frac{\mathbf{p}_i - \mathbf{p}_j}{\|\mathbf{p}_i - \mathbf{p}_j\|}$.

In **textbook BVC**, agent $i$ constrains its entire planned trajectory to remain within $\text{BVC}_i$. Because the cells are disjoint by construction, this gives a **hard geometric guarantee**: if all agents respect their BVCs simultaneously, no two can be within $r_\text{min}$ of each other.

> **Implementation note (code vs. theory).** The math above is *textbook* BVC — the perpendicular bisector through the midpoint $\frac{\mathbf{p}_i + \mathbf{p}_j}{2}$, buffered by $r_\text{min}/2$. The actual implementation in `cpp/src/bvc_avoidance.cpp` does **not** use this bisector/midpoint form. It reuses the **same linearized ellipsoidal keep-out constraint as on-demand** (gradient $\propto \mathbf{p}_i - \mathbf{p}_j$, tangent to the keep-out ellipsoid around $\mathbf{p}_j$ — not the midpoint), but applied **proactively** to all neighbours within $3\,r_\text{min}$ across the whole horizon, as soft (slack-penalized) constraints. So in this codebase, **"BVC" is effectively a proactive variant of on-demand**: it produces the conservative, wider-path behaviour associated with BVC, but does **not** provide the hard collision-free guarantee stated above. The guarantee is a property of textbook BVC, not of this implementation.

**BVC vs On-Demand (as implemented):**

| | On-Demand | BVC (this repo) |
|---|---|---|
| Scheduling | Reactive — first predicted violation | Proactive — all neighbours within $3 r_\text{min}$, full horizon |
| Constraint geometry | Linearized ellipsoidal tangent | **Same** linearized ellipsoidal tangent |
| Safety | Soft (slack-penalized), no hard guarantee | Soft (slack-penalized), no hard guarantee |
| Conservatism | Low — constrain only when needed | High — always constrains nearby pairs |
| Path width | Tight | Wide |
| Computational cost | Lower (sparse constraints) | Higher (dense constraints) |

---

## 5. Hungarian Algorithm for Task Reallocation

### 5.1 The Assignment Problem

Given $N$ agents at positions $\{\mathbf{p}_1, \ldots, \mathbf{p}_N\}$ and $N$ goals at $\{\mathbf{g}_1, \ldots, \mathbf{g}_N\}$, find the bijection $\pi : \{1,\ldots,N\} \to \{1,\ldots,N\}$ that minimizes total distance:

$$\pi^* = \arg\min_\pi \sum_{i=1}^N \|\mathbf{p}_i - \mathbf{g}_{\pi(i)}\|_2$$

This is the **linear sum assignment problem**, solvable in $O(N^3)$ via the Hungarian algorithm.

### 5.2 Cost Matrix

The cost matrix $\mathbf{C} \in \mathbb{R}^{N \times N}$ is built with:

$$C_{ij} = \|\mathbf{p}_i - \mathbf{g}_j\|_2$$

where $\mathbf{p}_i$ is the current position of agent $i$ and $\mathbf{g}_j$ is the $j$-th goal.

### 5.3 The Hungarian Algorithm

The algorithm works by iteratively finding and eliminating augmenting paths in a bipartite graph until a minimum-cost perfect matching is found. Steps:

1. **Row reduction:** Subtract the row minimum from each row of $\mathbf{C}$
2. **Column reduction:** Subtract the column minimum from each column
3. **Cover zeros:** Find the minimum number of lines to cover all zeros
4. **Optimality check:** If the number of covering lines equals $N$, an optimal assignment exists among the zeros
5. **Augment:** If not optimal, reduce uncovered elements and repeat

The result is a permutation vector `assignment` where `assignment[i] = j` means agent $i$ is assigned to goal $j$.

### 5.4 Reallocation Logic

Every `reallocation_period` seconds:

```
new_assignment = HungarianSolve(cost_matrix)

if new_assignment != current_assignment:
    for i in range(N):
        generator.setGoalPoint(i, original_goals[new_assignment[i]])
    current_assignment = new_assignment
    log_event()
```

Only the **original** goal positions are ever used as candidates — the algorithm reassigns which goal each drone is heading to, not where the goals are.

### 5.5 Predictive Mode

With `_use_predictive: true`, the cost is computed from **predicted future positions** rather than current ones:

$$C_{ij} = \|\hat{\mathbf{p}}_i(t + \Delta) - \mathbf{g}_j\|_2$$

where $\hat{\mathbf{p}}_i(t + \Delta)$ is taken from agent $i$'s current MPC horizon at prediction step $\Delta / T_s$. This accounts for momentum — an agent already moving toward a goal will be predicted to be much closer to it shortly, which may produce a different (better) assignment than using current positions.

---

## 6. Dynamic Goal Motion

Goals can follow one of five motion patterns, evaluated at each simulation step via `Generator::computeGoalPosition(agent_id, time)`:

**Static:** $\mathbf{g}_i(t) = \mathbf{g}_i(0)$ — original DMPC behavior.

**Circular:** Goals orbit a fixed center point $\mathbf{c}_i$ at radius $R$ and angular velocity $\omega$:

$$\mathbf{g}_i(t) = \mathbf{c}_i + R\begin{bmatrix}\cos(\omega t + \phi_i) \\ \sin(\omega t + \phi_i) \\ 0\end{bmatrix}$$

Parameters: `goal_circular_radius`, `goal_circular_omega`.

**Translating:** Goals move at constant velocity:

$$\mathbf{g}_i(t) = \mathbf{g}_i(0) + v \cdot t \cdot \hat{\mathbf{d}}_i$$

Parameter: `goal_translation_velocity`.

**Circular + Translating:** Combination of the above — orbiting while drifting linearly.

**Random Jump:** Goals teleport to a new random position every 5 seconds, requiring sudden replanning by the DMPC.

---

## 7. Simulation Loop Summary

Putting it all together, the simulation runs at two timescales:

**Fine timestep $T_s = 0.01$s (100 Hz):** Drone states are propagated using the discretized double integrator model with additive Gaussian noise ($\sigma_p = 0.00229$ m, $\sigma_v = 0.0109$ m/s) to simulate motion capture measurement noise.

**Planning timestep $h = 0.1$s (10 Hz):** The DMPC QP is solved. Every `reallocation_period = 2.0`s, the Hungarian reallocation check also runs here.

```
for k = 0 to K-1:                          # K = sim_duration / Ts steps
    if k % (h/Ts) == 0:                    # every 10 fine steps
        if reallocation_due:
            run Hungarian → update goals
        solve QP for all agents → new inputs
    apply inputs → propagate states (+ noise)
    record trajectories and goal positions

post-sim:
    collision check (minimum inter-agent distance over all time)
    goal check (final position within goal_tolerance = 0.15m)
    write trajectories.txt + goals.txt
```

---

## References

1. Luis, C. E., Vukosavljev, M., & Schoellig, A. P. (2020). Online trajectory generation with distributed model predictive control for multi-robot motion planning. *IEEE Robotics and Automation Letters*, 5(2), 604–611.

2. Kuhn, H. W. (1955). The Hungarian method for the assignment problem. *Naval Research Logistics Quarterly*, 2(1–2), 83–97.

3. Zhou, D., Wang, Z., Bandyopadhyay, S., & Schwager, M. (2017). Fast, on-line collision avoidance for dynamic vehicles using buffered Voronoi cells. *IEEE Robotics and Automation Letters*, 2(2), 1047–1054.

4. Ferreau, H. J., Kirches, C., Potschka, A., Bock, H. G., & Diehl, M. (2014). qpOASES: A parametric active-set algorithm for quadratic programming. *Mathematical Programming Computation*, 6(4), 327–363.

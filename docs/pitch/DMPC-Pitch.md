---
marp: true
theme: default
paginate: true
backgroundColor: #f8f9fa
style: |
  section {
    background-color: #f8f9fa;
    color: #212529;
    font-size: 28px;
    border-left: 8px solid #0066cc;
    padding-left: 60px;
  }
  h1 {
    color: #495057;
    font-size: 48px;
    border-bottom: 3px solid #0066cc;
    padding-bottom: 10px;
  }
  h2 {
    color: #0066cc;
    font-size: 36px;
  }
---

<!-- _class: lead -->
# **DMPC for Multi-Drone Motion Planning**
## *with Dynamic Goal Locations*

Distributed Model Predictive Control + gym-pybullet-drones

**Project Members:**
Shreyas Raorane | Kabir Puri

---

## **The Problem**

**Existing DMPC approaches assume:**
- Static goal locations for all agents
- Pre-defined end positions known at start
- No adaptation to moving targets

**Real-world scenarios need:**
- Dynamic targets (moving objects, changing missions)
- Real-time trajectory adaptation
- Collision-free paths with moving goals

---

## **Our Solution**

**Extending DMPC to handle dynamic goals using:**

1. **gym-pybullet-drones** simulation environment
   - Physics-based quadrotor dynamics
   - Real-time visualization
   - Validated drone models (Crazyflie 2.0)

2. **Moving goal adaptation**
   - Online replanning for goal updates
   - Event-triggered re-optimization
   - Maintain collision avoidance guarantees

---

## **Technical Approach**

**Core Components:**
- **Bézier curve parameterization** for smooth trajectories
- **On-demand collision avoidance** (50% faster than BVC method)
- **Event-triggered replanning** for disturbances & goal changes
- **Distributed QP optimization** solving at 5 Hz planning rate

**Key Innovation:** Adapting the optimization problem to track moving targets while preserving collision-free guarantees

<!-- 
---

## **Expected Impact**

**Simulation Results (from base paper):**
- ✓ 90%+ success rate with 30 agents in dense environments
- ✓ 50% reduction in travel time vs. traditional methods
- ✓ Real-time capable (20 Hz control with 20 drones)

**Our Extension:**
- Dynamic goal tracking in realistic physics simulator
- Demonstrate robustness to moving targets
- Validate scalability with gym-pybullet-drones -->

---

<!-- _class: lead -->
## **Next Steps**

1. Implement base DMPC in gym-pybullet-drones
2. Extend cost function for moving goal tracking
3. Test with varying goal dynamics (static → moving)
4. Benchmark performance & collision rates

**Goal:** Enable autonomous drones to handle dynamic mission objectives in real-time

---

<!-- _class: lead -->
# **Questions?**

**Reference Paper:**
Luis et al., "Online Trajectory Generation with Distributed Model Predictive Control for Multi-Robot Motion Planning"
*IEEE Robotics and Automation Letters, 2020*

https://ieeexplore.ieee.org/document/8950150


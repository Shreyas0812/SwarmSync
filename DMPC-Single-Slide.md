---
marp: true
theme: default
paginate: false
backgroundColor: #f8f9fa
style: |
  section {
    background-color: #f8f9fa;
    color: #212529;
    font-size: 20px;
    padding: 40px 50px;
  }
  h1 {
    color: #0066cc;
    font-size: 42px;
    border-bottom: 3px solid #0066cc;
    padding-bottom: 8px;
    margin-bottom: 15px;
  }
  h2 {
    color: #495057;
    font-size: 24px;
    margin-top: 15px;
    margin-bottom: 8px;
  }
  .columns {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 30px;
  }
  .box {
    background: white;
    padding: 15px;
    border-radius: 8px;
    border-left: 4px solid #0066cc;
  }
  ul {
    margin: 8px 0;
    padding-left: 20px;
  }
  li {
    margin: 4px 0;
    font-size: 18px;
  }
  strong {
    color: #0066cc;
  }
---

# **DMPC for Multi-Drone Motion Planning with Dynamic Goal Locations**

**Shreyas Raorane | Kabir Puri** | *Distributed Model Predictive Control + gym-pybullet-drones*

<div class="columns">

<div>

## **The Problem**
<div class="box">

**Current DMPC limitations:**
- Static goal locations only
- No moving target adaptation
- Pre-defined end positions

**Real-world needs:**
- Dynamic mission objectives
- Real-time replanning
- Collision-free paths with moving goals

</div>

## **Our Solution**
<div class="box">

**Extending DMPC framework:**
- **gym-pybullet-drones** simulation
- Physics-based quadrotor dynamics
- Moving goal adaptation
- Event-triggered re-optimization
- Collision avoidance guarantees

</div>

</div>

<div>

## **Technical Approach**
<div class="box">

**Core components from base paper:**
- **Bézier curve** trajectory parameterization
- **On-demand collision avoidance** (50% faster than BVC)
- **Event-triggered replanning** for disturbances
- **Distributed QP optimization** at 5 Hz planning rate

**Our innovation:** Adapt optimization for tracking moving targets while preserving safety

</div>

## **Expected Results**
<div class="box">

**Base paper achievements:**
- 90%+ success rate (30 agents, dense environments)
- 50% travel time reduction vs. traditional methods
- Real-time: 20 Hz control with 20 drones

**Our goal:** Dynamic goal tracking in realistic physics simulator with robustness validation

</div>

## **Next Steps**
1. Implement base DMPC in gym-pybullet-drones
2. Extend cost function for moving goals
3. Test with varying goal dynamics
4. Benchmark performance & collision rates

</div>

</div>

---

**Reference:** Luis et al., "Online Trajectory Generation with Distributed Model Predictive Control for Multi-Robot Motion Planning" *IEEE RA-L 2020*
https://ieeexplore.ieee.org/document/8950150

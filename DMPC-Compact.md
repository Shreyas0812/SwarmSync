---
marp: true
theme: default
paginate: false
backgroundColor: #f8f9fa
style: |
  section {
    background-color: #f8f9fa;
    color: #212529;
    font-size: 22px;
    padding: 35px 45px;
  }
  h1 {
    color: #0066cc;
    font-size: 40px;
    border-bottom: 3px solid #0066cc;
    padding-bottom: 6px;
    margin-bottom: 12px;
  }
  h2 {
    color: #495057;
    font-size: 26px;
    margin-top: 12px;
    margin-bottom: 6px;
  }
  .columns {
    display: grid;
    grid-template-columns: 1fr 1fr;
    gap: 25px;
  }
  .box {
    background: white;
    padding: 12px;
    border-radius: 6px;
    border-left: 4px solid #0066cc;
  }
  ul {
    margin: 5px 0;
    padding-left: 18px;
  }
  li {
    margin: 3px 0;
    font-size: 19px;
  }
  strong {
    color: #0066cc;
  }
---

# **DMPC for Multi-Drone Motion Planning with Dynamic Goals**

**Shreyas Raorane | Kabir Puri**

<div class="columns">

<div>

## **Problem & Solution**
<div class="box">

**Gap:** Current DMPC assumes static goals
**Innovation:** Extend to moving targets in **gym-pybullet-drones**

</div>

## **Technical Approach**
<div class="box">

- **Bézier curve** trajectories
- **On-demand collision avoidance** (50% faster than BVC)
- **Event-triggered replanning**
- **Distributed QP** at 5 Hz

**Extension:** Adapt optimization for dynamic goal tracking

</div>

</div>

<div>

## **Expected Results**
<div class="box">

**Base paper (Luis et al. RAL 2020):**
- 90%+ success, 30 agents
- 50% travel time reduction
- 20 Hz control with 20 drones

**Our goal:** Match performance with moving targets

</div>

## **Implementation**
<div class="box">

1. Base DMPC in simulator
2. Moving goal cost function
3. Performance benchmarking

</div>

</div>

</div>



**Reference:** Luis et al., "Online Trajectory Generation with Distributed Model Predictive Control for Multi-Robot Motion Planning" *IEEE RA-L 2020*
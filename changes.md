# Drone Fleet Management: Project Evolution & Solutions

This document details the progression of the Drone Fleet Simulator project, outlining the specific problems encountered at each stage and the algorithmic solutions applied to solve them.

## 1. Establishing the Foundation: The Baseline Simulator
**The Problem:**
Before building a complex, "smart" scheduler, we needed a reliable testing environment. The initial simulator only handled basic drone physics (movement, battery drain) for a single drone. We lacked a way to test fleet-wide behaviors, continuous package arrivals, and shared charging infrastructure.

**The Solution:**
- Scaled the simulator to a 10-drone fleet running concurrently.
- Introduced a central server logic (`assign_packages_baseline`) that blindly assigned the next available package to the first `IDLE` drone that could feasibly complete it.
- **Result:** The baseline worked but exposed severe operational flaws. Drone 1 ended up doing 90% of the work while Drone 10 sat idle, leading to massive fleet imbalance and uneven battery degradation.

---

## 2. Scheduler V1: Fleet Balance & Weighted Scoring
**The Problem:**
The baseline assignment algorithm was fundamentally unfair, causing uneven wear-and-tear on the fleet (the "fleet-imbalance behavior" warned about in the project specifications). 

**The Solution:**
- Replaced the blind assignment with a **Weighted Cost Scheduler (V1)**.
- For every new package, the scheduler evaluated all feasible idle drones.
- It calculated a score based on distance, deadline urgency, and a new **utilization penalty**.
- **Result:** The workload was successfully distributed across the 10 drones. However, running a stress test (64 packages in 10 minutes) revealed a deeper, more systemic problem: massive queues formed at the charging station, with drones waiting over 6 minutes just to access one of the 3 charging pads.

---

## 3. Identifying the True Bottleneck: Physical Constraints
**The Problem:**
We needed to know if our V1 algorithm was flawed, or if the system was physically starved. We ran experiments varying the number of charging pads (3, 5, 7, 10). 

**The Solution:**
- The data proved that with 10 pads, the V1 scheduler performed flawlessly with 0 wait time.
- This provided a critical insight: **The primary limiting factor of the fleet was not the assignment algorithm, but physical charging congestion.** The 3 charging pads were the true bottleneck.

---

## 4. The "Backfire" Insight: Expected Wait Time
**The Problem:**
We attempted to modify the scheduler to predict charging queue wait times and penalize assignments that would dump a drone into a traffic jam. Furthermore, we allowed drones to proactively charge if a pad was free before the rush hit.

**The Solution & Insight:**
- We built a forward-looking queue simulator.
- However, we discovered a fatal flaw in the standard charging logic: drones were programmed to charge to 100%. Because a full charge takes ~40 minutes, proactive charging caused 3 drones to seize the 3 pads right at the start of a 10-minute rush, permanently locking out the rest of the fleet. 
- **Result:** Package throughput plummeted. We learned that you cannot initiate a massive 30-minute charge cycle right before a 10-minute surge in demand.

---

## 5. Scheduler V2: Predictive Partial Charging
**The Problem:**
We needed an algorithm that actively managed the charging infrastructure as a constrained resource, rather than passively reacting to it. Drones needed to charge only what they needed, when they needed it.

**The Solution:**
We implemented a 3-stage receding-horizon fleet management algorithm:
1. **Continuous Charging Physics:** We rewrote the simulator so drones charge continuously per tick and automatically disconnect when they hit a dynamic target.
2. **Predictive Target (Stages 1 & 2):** For every idle/charging drone, the scheduler scans the pending package queue, identifies the most urgent feasible mission, calculates its required energy, and adds a flat 10% safety margin to define a custom `target_battery`.
3. **Infrastructure Management (Stage 3):** The scheduler actively kicks fully-charged drones off the pads and sorts waiting drones by how urgently they need power for their next predicted mission.

**The Final Result:**
Testing V2 against the Baseline under extreme charging contention yielded incredible improvements:

| Metric | Baseline | V2 (Predictive Partial) | Improvement |
| :--- | :---: | :---: | :---: |
| **Packages Delivered** | 42 | **63** | **+50%** |
| **Max Charging Wait** | 484.2s | **99.8s** | **-80%** |
| **Pad Utilization** | ~95% | **~38%** | **Massive relief** |

By abandoning the naive "charge to 100%" rule, V2 transformed the system into a resilient, highly efficient logistics network that easily handled surges despite severe physical constraints.

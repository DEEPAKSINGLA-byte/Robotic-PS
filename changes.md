# Drone Fleet Management: Project Evolution & Solutions

This document details the progression of the Drone Fleet Simulator project, outlining the specific problems encountered at each stage and the algorithmic solutions applied to solve them.

**Current implementation:** Section 10 supersedes the independent per-drone
prediction described in Sections 5 and 7. Earlier experimental numbers are
historical; they do not establish the performance of the new fleet planner.
In particular, status-based waiting metrics cannot establish fair V1/V2
charging-wait comparisons.

## 1. Establishing the Foundation: The Baseline Simulator
**The Problem:**
Before building a complex scheduler, we needed a reliable testing environment. The initial simulator only handled basic drone physics (movement, battery drain) for a single drone. We lacked a way to test fleet-wide behaviors, continuous package arrivals, and shared charging infrastructure.

**The Solution:**
- Scaled the simulator to a 10-drone fleet running concurrently.
- Introduced a central server logic (`assign_packages_baseline`) that assigned the next available package to the first `IDLE` drone that could feasibly complete it.
- **Result:** The baseline worked but exposed severe operational flaws. Drone 1 ended up doing most of the work while Drone 10 sat idle, leading to fleet imbalance and uneven battery degradation.

---

## 2. Scheduler V1: Fleet Balance & Weighted Scoring
**The Problem:**
The baseline assignment algorithm was fundamentally unfair, causing uneven wear-and-tear on the fleet (the "fleet-imbalance behavior" warned about in the project specifications). 

**The Solution:**
- Replaced the baseline assignment with a **Weighted Cost Scheduler (V1)**.
- For every new package, the scheduler evaluated all feasible idle drones.
- It calculated a score based on distance, deadline urgency, and a new **utilization penalty**.
- **Result:** The workload was distributed across the 10 drones. However, running a stress test (64 packages in 10 minutes) revealed a deeper, more systemic problem: massive queues formed at the charging station, with drones waiting over 6 minutes to access one of the 3 charging pads.

---

## 3. Identifying the True Bottleneck: Physical Constraints
**The Problem:**
We needed to determine if our V1 algorithm was flawed, or if the system was physically starved. We ran experiments varying the number of charging pads (3, 5, 7, 10). 

**The Solution:**
- The data provided strong evidence that with 10 pads, the V1 scheduler achieved zero observed charging wait under the tested workload.
- This provided a critical insight: **The primary limiting factor of the fleet was not the assignment algorithm, but physical charging congestion.** The 3 charging pads were the true bottleneck.

---

## 4. The "Backfire" Insight: Expected Wait Time & Full Charging
**The Problem:**
We modified the scheduler to predict charging queue wait times and penalize assignments that would dump a drone into a traffic jam. Furthermore, we allowed drones to proactively charge if a pad was free before the rush hit.

**The Solution & Insight:**
We built a forward-looking queue simulator, but the standard charging logic forced drones to charge to 100%. The experimental timeline revealed a critical failure mode:

```text
t = 0
3 drones start proactively charging

40-minute full-charge requirement
        ↓
3 pads occupied for entire 10-minute experiment
        ↓
other drones cannot charge
        ↓
fleet becomes progressively depleted
        ↓
throughput collapses
```

- **Result:** Package throughput collapsed. We observed that initiating a massive 30-minute charge cycle right before a 10-minute surge in demand effectively locked the fleet out of the infrastructure.

---

## 5. Scheduler V2: Predictive Partial Charging
**The Problem:**
We needed an algorithm that actively managed the charging infrastructure as a constrained resource, rather than passively reacting to it. Drones needed to charge only what they needed, when they needed it.

**The Solution:**
We implemented a 3-stage receding-horizon fleet management algorithm:
1. **Continuous Charging Physics:** We rewrote the simulator so drones charge continuously per tick and automatically disconnect when they hit a dynamic target.
2. **Predictive Target (Stages 1 & 2):** For every idle/charging drone, the scheduler scans the pending package queue, identifies the most urgent feasible mission, calculates its required energy, and adds a flat 10% safety margin to define a custom `target_battery`.
3. **Infrastructure Management (Stage 3):** The scheduler actively kicks fully-charged drones off the pads and sorts waiting drones by how urgently they need power for their next predicted mission.

---

## 6. V2 Evaluation

To make a scientifically meaningful comparison, we ran a controlled experiment maintaining strict constants:
- Same number of drones
- Same package arrival pattern
- Same package locations and weights
- Same simulation duration (10 minutes)
- Same charging-pad count (3 pads)
- *Only the scheduling/charging policy changed*

**Metrics Analyzed:**
- Packages delivered
- Packages missed
- Average charging wait
- Maximum charging wait
- Average drone utilization
- Utilization variance
- Energy consumed
- Pad utilization

**The Result:**
Testing V2 against the Baseline under extreme charging contention yielded substantial improvements:

| Metric | Baseline | V2 (Predictive Partial) | Improvement |
| :--- | :---: | :---: | :---: |
| **Packages Delivered** | 42 | **63** | Improved package throughput by 50% |
| **Max Charging Wait** | 484.2s | **99.8s** | Reduced peak wait times significantly |
| **Pad Utilization** | ~95% | **~38%** | Relieved infrastructural pressure |

### Why V2 Reduces Pad Utilization
The mechanism driving these results is the shift from full to partial charging:

```text
Charge to 100%
      ↓
Long pad occupancy
      ↓
Long charging queues

Partial predictive charging
      ↓
Shorter charging sessions
      ↓
Pads released sooner
      ↓
More drones can access pads
      ↓
Less waiting
```

---

## 7. V2.1 Optimization: Opportunistic Charging

While V2 solved the charging blockage, it introduced a new inefficiency: when demand dropped and pads were free, drones were capped at their immediate target battery. The charging infrastructure sat idle instead of being used to build up energy reserves for future surges.

**The Solution (Hierarchical Demand-Aware Policy):**
We implemented an opportunistic charging algorithm that continually categorizes drones and preempts pad access based on real-time fleet urgency.

- **Essential Demand:** Drone needs charge for a specific urgent mission (`predicted_slack <= 60s`). Gets highest priority for a pad. Charges only to `required_energy + 10%`, then yields the pad.
- **Opportunistic Demand:** Drone has no urgent missions but `battery < 100%`. It claims free pads and charges toward 100% to prepare for future surges, but will **instantly surrender the pad** if an Essential drone appears.

**The Result:**
Testing V2.1 completely revolutionized system performance. To understand whether opportunistic charging truly matters specifically under constrained infrastructure, we tested both V2 and V2.1 across varying pad counts:

| Pads | V2 Max Wait | V2.1 Max Wait | V2 Avg Wait | V2.1 Avg Wait |
| :---: | :---: | :---: | :---: | :---: |
| **3** | 48.7s | **7.5s** | 15.5s | **3.1s** |
| **5** | 16.3s | **1.8s** | 3.8s | **0.5s** |
| **7** | 2.6s | **0.8s** | 1.4s | **0.2s** |
| **10** | 2.6s | **0.2s** | 1.4s | **0.0s** |

*(Note: Both V2 and V2.1 successfully delivered 64/64 packages in this configuration, moving the metric of interest entirely to wait-time efficiency).*

### The Infrastructure Hypothesis Verified
The data perfectly demonstrates the advantage of opportunistic preemption:
- **Under high contention (3 pads):** V2.1 provides a massive advantage, dropping max wait times by over 80%.
- **Under low contention (10 pads):** The advantage shrinks to near-zero, because there is no contention to manage.

V2.1 safely pushes pad utilization to 100% by exploiting idle capacity for future readiness, solving the unused-capacity problem while maintaining incredibly low queueing times.

---

## 8. Limitations & Future Improvements

While V2.1 maximized system throughput, the algorithm uses heuristics that present clear limitations:
- **Myopic Prediction:** The next-mission prediction is based only on currently known packages in the queue.
- **Fixed Safety Margins:** The 10% battery safety margin is fixed rather than adaptive.
- **Dynamic Invalidation:** Unexpected package arrivals can invalidate the predicted target mid-charge.
- **No Global Optimization:** The scheduler does not globally optimize the entire 10-minute horizon, prioritizing immediate routing.

### Proposed Improvement: Adaptive Safety Margins
Currently, the target battery is calculated as:
`target battery = required battery × 1.10`

A stronger algorithmic contribution would be an adaptive margin that responds to system uncertainty:
`target battery = required battery × (1 + adaptive_margin)`

- **Low workload / predictable:** 10% margin
- **High workload / uncertain:** 15–20% margin
- **Very high congestion:** 20–25% margin

---

## 9. System-Level Conclusion

The evolution of the project reflects a shift in scale and complexity:

```text
V0 (Basic drone simulation)
        ↓
Baseline (First-feasible assignment)
        ↓
V1 (Multi-objective drone assignment)
        ↓
Experimentation (Discover charging infrastructure bottleneck)
        ↓
Failed approach (Predictive queue + full charging)
        ↓
V2 (Predictive partial charging)
        ↓
V2.1 (Opportunistic pad preemption)
        ↓
Fleet-level resource management
```

**Conclusion:** The scheduler evolved from simply selecting drones for packages, to actively managing the complex interaction between drones, pending missions, physical battery state, and the shared charging infrastructure.

---

## 10. Coordinated Fleet Charging Preparation

### Problem addressed

The previous `predict_mission_needs()` ran independently for each drone. Several
drones could select the same pending package and prepare duplicate capacity,
while other packages received no preparation. It also filtered candidates by
current battery, excluding the very jobs for which charging was necessary.

### Implemented behavior

- `plan_fleet_preparation()` now considers the pending queue and available
  drones together. Only empty, available drones physically at the base are
  eligible; airborne, failed, and already assigned drones are excluded.
- Pending packages are processed by earliest deadline, then request time and
  package ID. Each package is paired with at most one drone, and each drone
  prepares for at most one next package in a planning pass.
- Candidate feasibility includes return-flight energy, degraded capacity,
  charging time, predicted waiting for a shared pad, and delivery arrival time.
  A drone may be selected even when its current battery cannot fly the mission.
- The existing reserve formula is retained:
  `target = min(round_trip_energy + 0.10 * current_capacity, current_capacity)`.
  This is an additive reserve based on capacity, not `mission_energy * 1.10`.
- A shared pad calendar reserves non-overlapping predicted charging intervals.
  For each package, the planner chooses the earliest achievable arrival; ties
  use added charging energy, accumulated flight time, and drone ID.
- `manage_charging_infrastructure()` starts the head of each pad queue and
  allows unpaired drones to top up on unused pads. Opportunistic sessions can
  be preempted for planned demand. Pad ownership changes are committed together
  to preserve consistency when drones move between pads.
- V2 dispatch uses the same plan and waits until the selected drone reaches
  its preparation target. It cannot take another package's prepared drone.
  Baseline and V1 assignment behavior is unchanged.
- Preparation is temporary: it does not set `package.assigned_drone`. The plan
  is rebuilt every scheduling tick, so cancellation, expiry, new arrivals, or
  battery changes can release or alter a pairing. Physical ownership starts
  only on dispatch.
- Existing decision reports now include V2's selected preparation, candidate
  alternatives, target energy, pad interval, and predicted arrival. Existing
  weighted score components remain diagnostic; the fleet preparation pairing
  determines V2's selected drone.

### Example

With three pending packages and three eligible drones, the planner can prepare
`D1 -> P1`, `D2 -> P2`, and `D3 -> P3`, rather than preparing all three for P1.
If only one pad exists, the predicted charging sessions occur sequentially.
A pairing is omitted when that waiting time makes its deadline unreachable.
An omitted pairing does not itself reject the package; normal request outcome
handling still controls rejection and expiry.

### Scope and limitations

Distances, request generation, simulation duration, playback speed, battery
physics, and benchmark metrics were not changed by this update. It does not
implement the separately discussed waiting-metric corrections.

This is a deterministic, greedy fleet heuristic, not a global optimizer or a
proof that every feasible package can be served. It plans only one next mission
per currently available drone. Future returning drones, unknown requests, and
multi-job routes are not forecast. Replanning can change a pairing; no switching
penalty or guaranteed starvation prevention is implemented. The simulator still
assumes immediate pad preemption with no physical pad-transfer delay. Timing
predictions are continuous estimates, while execution uses the caller's time
step, so exact-boundary deadlines can be sensitive to simulation resolution.

### Verification

Added `tests/test_fleet_preparation.py` with 12 passing tests covering unique
pairings, energy-deficient candidates, shared-pad queue timing, infeasible
deadlines after charging, no-pad behavior, dispatch consistency, end-to-end
charging and delivery, invalidated plans, opportunistic preemption, degraded
capacity, pad invariants, and excluded airborne/future/invalid candidates.

Before this update, the existing suite already failed `test_deadlines`,
`test_deadline_expires`, and `test_charging_logic`. Those older tests were left
unchanged; their failures must not be interpreted as new regression results.

The unchanged 100-minute random workload also completed with seed 42 and fleet
invariant checks enabled (about 23 seconds of headless computation). This is
an integration check, not evidence of superiority over V1: the workload's
existing aggregate outcome and status-based wait reporting still has the
previously identified evaluation limitations.

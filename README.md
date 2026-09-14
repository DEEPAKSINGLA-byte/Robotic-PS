
# Drone Fleet 2D Simulator

A lightweight custom OpenCV simulator for the Inter-IIT Drone Fleet Management problem.

The problem statement explicitly allows a custom 2D simulator with OpenCV if a flight simulator becomes a time sink.

## What is implemented

- 10 drones
- 1 central base
- 3 charging pads
- Package weight: 0.2–2.5 kg
- Cruise speed model: 12 m/s, reduced with payload
- Battery model
- Battery degradation per charge cycle
- Approximate 25-minute full-load endurance
- Approximate 40-minute full recharge
- Continuous package generation
- Deadline feasibility checks
- Centralized drone assignment
- Fleet balancing term in assignment score
- Return-to-base after delivery
- Charging-pad contention
- Communication-failure injection
- Re-queue of a package after drone failure
- Live fleet portal
- Event log
- Delivery/late/pending/charging/failure/distance statistics

## Install

Python 3.10+ is recommended.

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Run

```bash
python3 simulation.py
```

This runs 40 simulated minutes with 24x playback (about 100 real seconds,
subject to machine performance). Use `--time-scale 20` for approximately
two real minutes, or `--headless` to run as fast as the computer allows.
For example: `python3 simulation.py --headless --minutes 40 --algorithm v1`.
Algorithms: `baseline`, `v1`, `v2` (default).

All movement, battery use, charging, deadlines, and request arrivals use
simulated seconds. Playback changes only how quickly those seconds pass.
The physical model retains 25-minute full-load endurance and 40-minute
empty-to-full charging. The demo uses the existing short-distance stress
workload with 60-second deadlines, not the statement's 10–45 minute workload.
`drone_sim.py` alone remains a passive renderer.

Each run writes a unique JSON report under `logs/` (`--log-dir` overrides it).
Reports contain outcomes, actual arrival times, assignment/defer decisions,
candidate rejection reasons, and weighted score components for V1/V2.
The baseline records its first-feasible-idle selection rule rather than a
weighted score. All idle drones are considered before it defers a request.
The hard and random workload scripts also save reports and print separate
on-time, late, rejected, expired, pending, and in-flight counts.

Pending requests are rejected for invalid/overweight input or when even an
immediate fully charged departure from the base cannot complete them safely
and on time. Temporary battery shortages or busy drones cause deferral.
Unassigned requests whose deadline passes expire. Airborne deliveries retain
their owner and become on-time or late on arrival; deadlines do not teleport
or reassign their packages.

## Reproducible policy comparison

Run `python3 run_random_workload.py` to compare Baseline, V1 and V2 on the same
100-minute workloads for seeds 42, 100 and 2026. Use `--seeds 42` for one seed,
or `--no-decision-logs` for lighter evaluation. Reports are written under
`logs/`; `--log-dir PATH` selects another folder.

All policies use the same full 10%-of-current-capacity return reserve. Reports
separate all six delivery outcomes and record low-battery waiting episodes only
for empty drones at base without a pad. Multi-seed summaries include sample
standard deviations and the worst observed episode, including unfinished ones.

V2 forecasts up to three jobs per drone and ranks delivery coverage followed by
charging/flight resource use. After 600 seconds of low-battery waiting, it gives
recovery charging priority. This is an intervention threshold, not a guaranteed
maximum wait under overload. See `changes.md`, Sections 12–13, for details.

## V2 delivery-focused improvements

These four changes affect V2 scheduling only. Fleet size, pad count, distances,
arrival/deadline distributions, battery physics, simulation duration and playback
speed are unchanged. Every policy still requires energy for the loaded delivery,
empty return, and the same full 10%-of-current-capacity safety reserve.

1. **Recovery charging prepares a delivery.** After a continuous low-battery wait
   reaches 600 simulated seconds, the recovery scheduler first looks for a known
   pending package that the drone can safely deliver on time. Each recovery drone
   receives a different package and charges only to that mission's safe departure
   target. An already-ready drone can depart immediately, even below 20%, without
   taking a pad. The pairing stays preferred while feasible and is revalidated
   each tick; an expired or otherwise unavailable package releases it. If no
   suitable package exists, recovery falls back to 20% of current capacity.

2. **Try alternate delivery orders.** Within the three-job-per-drone horizon,
   each new candidate is tried at every position in the retained route. A short
   delivery can therefore move ahead of a long one when doing so allows more
   packages to arrive on time. Each trial recalculates charging slots, delivery
   and return times, battery use and degradation. Other drones' reservations
   remain protected. Only the first job is executable; later jobs are forecasts,
   not permanent package assignments.

3. **Keep meaningfully different candidate plans.** The six-plan beam removes
   exact duplicates, always retains the best current plan and an alternative
   that skips the newly considered package, then favors competitive alternatives
   with different route orders or charging profiles. Simply swapping equivalent
   drone/pad labels does not count as a new strategy. Diversity cannot displace
   the best plan. This remains a bounded heuristic: pruning can miss a better
   ordering or assignment, and global optimality is not guaranteed.

4. **Use spare charging time to make another drone useful sooner.** Recovery and
   planned missions take priority. Remaining pads favor drones needing the least
   additional charging time to reach a useful departure target, rather than
   automatically choosing the lowest battery. Targets come first from uncovered
   feasible pending packages; otherwise they use the median safe mission energy
   of up to 50 already-observed recent requests. Without usable history, the
   fallback is 20%. Drones below their readiness target precede top-ups of already
   ready drones, and opportunistic charging yields to future pad reservations.
   Recent demand is only an estimate, never knowledge of future requests or
   permission to depart without a fresh safety/deadline check.

Decision reports include the charging basis and preparation sequence for package
evaluations. Optional charging does not permanently reserve a package. This
policy still trades some throughput for starvation protection; reduced waiting
alone is not evidence of improved deliveries. Compare paired seeds before drawing
conclusions, and do not treat three seeds as statistical proof.

Run the focused regression suite with:

```bash
python3 -B -m unittest discover -s tests -v
```

The suite includes recovery completion and cancellation, safe sub-20% dispatch,
alternate ordering, shared-pad non-overlap, beam diversity, and no use of future
requests in opportunistic predictions. The historical function-style tests in
`tests/test_simulation.py` are separate from this unittest suite; three pre-existing
failures concerning deadline and charging expectations remain unresolved.

### Measured result after these changes

On the unchanged 100-minute workloads for seeds 42, 100 and 2026:

| Policy | On-time deliveries, mean ± sample SD | Worst observed low-battery wait |
| --- | ---: | ---: |
| Baseline | 116.67 ± 1.53 | 5741 s |
| V1 | 221.33 ± 1.53 | 5648 s |
| Updated V2 | 219.33 ± 1.15 | 600 s |

All nine runs had zero late deliveries and zero failed drones. V2's on-time
counts were 220, 220 and 218, versus 220, 221 and 219 before these four changes.
The targeted scenarios demonstrate the new behaviors, but this stress workload
does **not** show a throughput improvement. The observed wait bound is not a
guarantee. No parameters were tuned after seeing these results.

The separate 40-minute demo completed headlessly in 14.1 wall-clock seconds on
the test machine, with all 240 requests on time. This checks the simulation path,
not graphical rendering performance or a guaranteed runtime on other machines.

## Controls

- `Q` / `ESC` — quit

## Important design point

The OpenCV renderer is separate from the fleet simulation logic. The simulator is therefore usable as a baseline for the actual assignment/charging algorithms.

Suggested next steps:

1. Move assignment logic into `algorithms/assignment.py`.
2. Move charging logic into `algorithms/charging.py`.
3. Add deterministic scenario files for benchmarking.
4. Log every assignment decision and its score components.
5. Add CSV output for delivery rate, lateness, distance, energy, idle time and pad utilisation.
6. Add the edge cases from the problem statement.
7. Add a multi-run benchmark comparing assignment algorithms.

## Current assignment score

The baseline considers:

- feasibility / ability to complete the job and return
- distance
- battery reserve
- deadline slack
- workload balancing

This is intentionally a simple baseline, not the final optimization algorithm.
# Swarm_drones

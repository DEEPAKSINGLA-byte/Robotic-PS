
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

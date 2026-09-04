
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
python3 drone_sim.py
```

## Controls

- `SPACE` — pause/resume
- `N` — create a package immediately
- `F` — inject a drone communication failure
- `A` — toggle automatic package generation
- `R` — reset
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

import math
import argparse
import json
import time
from pathlib import Path
import cv2
from drone_sim import DroneState, PackageState, BASE, Drone2DEnvironment


from physics import *
from charging import *
from assignment_costs.features import *
from assignment_costs.v1_cost import *
from assignment_costs.v2_cost import *
from assignment_costs.baseline_cost import *


def should_create_hard_package(sim_time):
    # Harder workload: faster arrivals (every 10s)
    return sim_time > 0 and (sim_time % 10.0) < 0.001

def generate_hard_package(package_id, sim_time):
    # More dispersed drop points, heavy weights, shorter deadlines
    dx = [300, -400, 500, -200, 600, -500, 250, -350, 450, -600]
    dy = [400, 300, -500, -200, 200, 600, -300, -400, 150, -50]
    idx = package_id % 10
    x = BASE[0] + dx[idx]
    y = BASE[1] + dy[idx]
    # Heavy and light
    weight = 0.5 if (package_id % 2 == 0) else 2.5 
    # Short deadline (between 60s to 120s depending on distance)
    # The absolute distance is up to 850px (~85 meters). Round trip ~170m.
    # At 8.4m/s, flight takes ~20 seconds.
    deadline = sim_time + 60.0
    
    return PackageState(
        id=package_id,
        x=float(x),
        y=float(y),
        weight=weight,
        request_time=sim_time,
        deadline=deadline,
        assigned_drone=None,
        delivered=False
    )

def assign_package(drone, package):
    package.status = "ASSIGNED"
    drone.status = "DELIVERY"
    drone.target = (package.x, package.y)
    drone.payload = package.weight
    drone.current_package = package.id
    package.assigned_drone = drone.id
    
def get_feasible_drones(drones, package, sim_time):
    feasible = []

    for drone in drones.values():
        if drone.status not in ("IDLE", "WAITING_FOR_CHARGE", "CHARGING"):
            continue

        if is_mission_feasible(drone, package, sim_time):
            feasible.append(drone)

    return feasible
def update_drone(drone, packages, dt, charging_pads, sim_time=0.0):

    remaining_dt = dt

    while remaining_dt > 0:

        speed_meters = calculate_speed(drone.payload)
        speed_pixels = meters_to_pixels(speed_meters)

        if drone.status == "DELIVERY":
            if drone.current_package is None:
                return
            package = packages[drone.current_package]

            distance_moved, actual_time, reached = move_towards(
                drone,
                drone.target,
                speed_pixels,
                remaining_dt
            )
            
            drone.total_distance += distance_moved
            drone.total_flight_time += actual_time

            consume_battery(drone, actual_time)

            remaining_dt -= actual_time

            if drone.battery <= 0:
                drone.battery = 0.0
                drone.status = "FAILED"
                drone.target = None
                return

            if reached:
                package.delivered = True
                package.delivered_at = sim_time + dt - remaining_dt
                package.status = ("DELIVERED_ON_TIME" if package.delivered_at <= package.deadline + 1e-9
                                  else "DELIVERED_LATE")
                package.outcome_reason = None if package.status == "DELIVERED_ON_TIME" else "Arrival after deadline"

                drone.payload = 0.0
                drone.status = "RETURNING"
                drone.target = (BASE[0], BASE[1])

                continue

        elif drone.status == "RETURNING":

            distance_moved, actual_time, reached = move_towards(
                drone,
                drone.target,
                speed_pixels,
                remaining_dt
            )
            
            drone.total_distance += distance_moved
            drone.total_flight_time += actual_time

            consume_battery(drone, actual_time)

            remaining_dt -= actual_time

            if drone.battery <= 0:
                drone.battery = 0.0
                drone.status = "FAILED"
                drone.target = None
                return

            if reached:
                drone.target = None
                drone.payload = 0.0

                if drone.battery < 0.20 * drone.battery_capacity:
                    drone.status = "WAITING_FOR_CHARGE"
                else:
                    drone.status = "IDLE"

                drone.current_package = None

                continue

        elif drone.status == "CHARGING":
            update_charging(drone, remaining_dt, charging_pads)
            return

        elif drone.status == "WAITING_FOR_CHARGE":
            drone.total_charge_wait_time += remaining_dt
            if start_charging(drone, charging_pads):
                return
            return

        else:
            return












def validate_simulation(drones, charging_pads):
    for drone in drones.values():
        assert drone.battery >= 0.0, f"Drone {drone.id} battery negative: {drone.battery}"
        assert drone.battery <= drone.battery_capacity + 1e-6, f"Drone {drone.id} battery exceeds capacity"

        if drone.status == "CHARGING":
            assert drone.charging_pad is not None, f"Drone {drone.id} charging but no pad"
            assert charging_pads[drone.charging_pad] == drone.id, f"Pad {drone.charging_pad} doesn't match drone {drone.id}"
        else:
            assert drone.charging_pad is None, f"Drone {drone.id} not charging but has pad {drone.charging_pad}"

    for pad_id, drone_id in charging_pads.items():
        if drone_id is not None:
            assert drone_id in drones
            drone = drones[drone_id]
            assert drone.status == "CHARGING", f"Pad {pad_id} has drone {drone_id} but status is {drone.status}"
            assert drone.charging_pad == pad_id, f"Drone {drone_id} on pad {pad_id} thinks it's on {drone.charging_pad}"

def get_idle_drone(drones):
    for drone in drones.values():
        if drone.status == "IDLE":
            return drone
    return None

def update_package_outcomes(drones, packages, sim_time):
    """Only unassigned jobs expire; an airborne package keeps its physical owner."""
    for package in packages.values():
        if package.status != "PENDING" or package.assigned_drone is not None or package.delivered:
            continue
        if package.request_time > sim_time:
            continue
        values = (package.x, package.y, package.weight, package.deadline, package.request_time)
        reason = None
        if not all(math.isfinite(v) for v in values) or package.weight < 0:
            reason = "Invalid request"
        elif package.weight > 2.5:
            reason = "Payload exceeds fleet limit"
        elif sim_time > package.deadline:
            package.status = "EXPIRED"
            reason = "Deadline passed while waiting"
        elif drones:
            # Packages are collected at the base. Ignore commitments here:
            # reject only if even an immediately available, fully charged drone cannot serve it.
            ideal = DroneState(id=0, x=float(BASE[0]), y=float(BASE[1]))
            if sim_time + calculate_delivery_time(ideal, package) > package.deadline + 1e-9:
                reason = "Even immediate departure cannot meet deadline"
            elif calculate_required_battery(ideal, package) > max(d.battery_capacity for d in drones.values()):
                reason = "Round trip exceeds every drone's full usable capacity"
        if reason:
            if package.status != "EXPIRED":
                package.status = "REJECTED"
            package.outcome_reason = reason
            package.assigned_drone = -1
            package.decision_history.append({
                "sim_time": sim_time, "action": package.status,
                "reason": reason, "candidates": [],
            })


def candidate_details(drone, package, sim_time, allowed):
    reasons = []
    if drone.status not in allowed:
        reasons.append("Drone unavailable: " + drone.status)
    if package.weight > 2.5:
        reasons.append("Payload exceeds capacity")
    arrival = sim_time + calculate_delivery_time(drone, package)
    energy = calculate_required_battery(drone, package)
    if arrival > package.deadline + 1e-9:
        reasons.append("Arrival after deadline")
    if energy > drone.battery:
        reasons.append("Insufficient battery for delivery and return")
    return {
        "drone_id": drone.id, "status": drone.status, "battery": drone.battery,
        "capacity": drone.battery_capacity, "arrival": arrival,
        "required_energy": energy, "reasons": reasons,
    }


def schedule_packages(drones, packages, sim_time, charging_pads, algorithm):
    update_package_outcomes(drones, packages, sim_time)
    preparation = {}
    if algorithm == "v2":
        preparation = manage_charging_infrastructure(drones, packages, charging_pads, sim_time)
    package_plans = {plan["package_id"]: plan for plan in preparation.values()}
    allowed = ("IDLE",) if algorithm == "baseline" else ("IDLE", "WAITING_FOR_CHARGE", "CHARGING")
    for package in packages.values():
        if package.status != "PENDING" or package.assigned_drone is not None or package.delivered:
            continue
        if package.request_time > sim_time:
            continue
        candidates = [candidate_details(d, package, sim_time, allowed) for d in drones.values()]
        plan = package_plans.get(package.id)
        if algorithm == "v2":
            for candidate in candidates:
                if plan is None or candidate["drone_id"] != plan["drone_id"]:
                    candidate["reasons"].append("Not paired with this package in fleet preparation")
                elif drones[candidate["drone_id"]].battery + 1e-9 < plan["target_battery"]:
                    candidate["reasons"].append("Waiting for mission energy plus preparation reserve")
        feasible = [c for c in candidates if not c["reasons"]]
        selected = None
        if feasible:
            if algorithm == "baseline":
                selected = feasible[0]  # First feasible idle drone, not first idle drone.
                reason = "First feasible idle drone in fleet order"
            else:
                features = {
                    c["drone_id"]: calculate_assignment_features(drones, c["drone_id"], package, sim_time)
                    for c in feasible
                }
                max_distance = max(f["raw_distance"] for f in features.values())
                avg_time = sum(d.total_flight_time for d in drones.values()) / len(drones)
                score_fn = calculate_assignment_score if algorithm == "v1" else calculate_assignment_score_v2
                for candidate in feasible:
                    candidate["features"] = features[candidate["drone_id"]]
                    candidate["score_components"] = score_fn(
                        features[candidate["drone_id"]], max_distance, avg_time)
                selected = min(feasible, key=lambda c: c["score_components"]["score"])
                reason = ("Fleet preparation ready; earliest feasible arrival in deadline order"
                          if algorithm == "v2" else "Lowest weighted score; fleet order breaks ties")
        else:
            reason = "No drone available with sufficient time and return energy; retry later"
        package.decision_history.append({
            "sim_time": sim_time, "algorithm": algorithm,
            "action": "ASSIGNED" if selected else "DEFERRED",
            "selected_drone": selected["drone_id"] if selected else None,
            "reason": reason, "candidates": candidates,
            "preparation": plan,
        })
        if selected:
            drone = drones[selected["drone_id"]]
            if drone.charging_pad is not None:
                charging_pads[drone.charging_pad] = None
                drone.charging_pad = None
            assign_package(drone, package)


def summarize_packages(packages):
    counts = {status: 0 for status in (
        "PENDING", "ASSIGNED", "REJECTED", "EXPIRED", "DELIVERED_ON_TIME", "DELIVERED_LATE")}
    for package in packages.values():
        counts[package.status] += 1
    return counts


def save_run_report(packages, directory="logs", label="simulation"):
    """Unique run files preserve previous experiments and candidate explanations."""
    from datetime import datetime, timezone
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    path = directory / (label + "-" + stamp + ".json")
    records = [{
        "package_id": p.id, "request_time": p.request_time, "deadline": p.deadline,
        "status": p.status, "assigned_drone": p.assigned_drone,
        "delivered_at": p.delivered_at, "reason": p.outcome_reason,
        "decisions": p.decision_history,
    } for p in packages.values()]
    with path.open("x") as stream:
        json.dump({"summary": summarize_packages(packages), "packages": records},
                  stream, indent=2)
    return path


def assign_packages_baseline(drones, packages, sim_time, charging_pads):
    schedule_packages(drones, packages, sim_time, charging_pads, "baseline")


def assign_packages_v1(drones, packages, sim_time, charging_pads):
    schedule_packages(drones, packages, sim_time, charging_pads, "v1")






def assign_packages_v2(drones, packages, sim_time, charging_pads):
    schedule_packages(drones, packages, sim_time, charging_pads, "v2")

def validate_fleet(drones, charging_pads, packages):
    validate_simulation(drones, charging_pads)

    for drone in drones.values():
        if drone.current_package is not None:
            package = packages[drone.current_package]
            assert package.assigned_drone == drone.id, f"Drone {drone.id} carries package {package.id} but package thinks it's assigned to {package.assigned_drone}"

    for package in packages.values():
        if package.assigned_drone not in (None, -1):
            assert package.assigned_drone in drones, f"Package {package.id} assigned to invalid drone {package.assigned_drone}"

def run_simulation_step(drone, packages, dt, sim_time, charging_pads):
    update_drone(drone, packages, dt, charging_pads, sim_time)

def initialize_drones(num_drones=10):
    drones = {}
    for i in range(1, num_drones + 1):
        drones[i] = DroneState(
            id=i,
            x=float(BASE[0]),
            y=float(BASE[1]),
            battery=100.0,
            battery_capacity=100.0,
            cycle_count=0,
            cycle_energy=0.0,
            status="IDLE",
            payload=0.0,
            target=None,
            total_distance=0.0,
            charging_pad=None,
            charge_remaining=0.0
        )
    return drones

DEFAULT_TIME_SCALE = 24.0  # 40 simulated minutes in about 100 real seconds.
SIMULATION_STEP = 0.25


def run_demo(duration=40 * 60, time_scale=DEFAULT_TIME_SCALE, headless=False,
             algorithm="v2", log_dir="logs"):
    drones = initialize_drones()
    packages = {}
    charging_pads = {1: None, 2: None, 3: None}
    scheduler = {"baseline": assign_packages_baseline, "v1": assign_packages_v1,
                 "v2": assign_packages_v2}[algorithm]
    env = None if headless else Drone2DEnvironment(num_drones=10)
    if env:
        cv2.namedWindow("Drone 2D Environment", cv2.WINDOW_NORMAL)
    sim_time = 0.0
    next_request = 0.0
    wall_start = time.monotonic()
    try:
        while sim_time < duration:
            # Wall time only controls playback. All physical logic uses simulated seconds.
            target_time = duration if headless else min(duration, (time.monotonic() - wall_start) * time_scale)
            while sim_time < target_time:
                dt = min(SIMULATION_STEP, duration - sim_time)
                if sim_time + 1e-9 >= next_request:
                    package = generate_hard_package(len(packages) + 1, sim_time)
                    packages[package.id] = package
                    next_request += 10.0
                scheduler(drones, packages, sim_time, charging_pads)
                for drone in drones.values():
                    run_simulation_step(drone, packages, dt, sim_time, charging_pads)
                sim_time += dt
                update_package_outcomes(drones, packages, sim_time)
                validate_fleet(drones, charging_pads, packages)
            if env:
                env.update_state(drones, packages, list(charging_pads.values()),
                                 message=f"{sim_time / 60:.1f} sim min | {time_scale:g}x playback")
                cv2.imshow("Drone 2D Environment", env.draw())
                if cv2.waitKey(1) & 0xFF in (ord("q"), ord("Q"), 27):
                    break
    finally:
        path = save_run_report(packages, log_dir, algorithm)
        print("Outcomes:", summarize_packages(packages))
        print("Decision report:", path)
        if env:
            cv2.destroyAllWindows()
    return drones, packages


def main():
    parser = argparse.ArgumentParser(description="Accelerated drone fleet simulation")
    parser.add_argument("--minutes", type=float, default=40.0)
    parser.add_argument("--time-scale", type=float, default=DEFAULT_TIME_SCALE)
    parser.add_argument("--headless", action="store_true", help="Run without graphics or playback delays")
    parser.add_argument("--algorithm", choices=("baseline", "v1", "v2"), default="v2")
    parser.add_argument("--log-dir", default="logs")
    args = parser.parse_args()
    if not math.isfinite(args.minutes) or args.minutes <= 0 or not math.isfinite(args.time_scale) or args.time_scale <= 0:
        parser.error("minutes and time-scale must be finite positive numbers")
    run_demo(args.minutes * 60, args.time_scale, args.headless, args.algorithm, args.log_dir)


if __name__ == "__main__":
    main()
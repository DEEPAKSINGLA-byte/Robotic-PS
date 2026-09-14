import math
import argparse
import json
import time
import statistics
from pathlib import Path
from dataclasses import replace
import cv2
from drone_sim import DroneState, PackageState, BASE, Drone2DEnvironment

def move_towards(drone, target, speed, dt):
    target_x, target_y = target

    dx = target_x - drone.x
    dy = target_y - drone.y

    distance = math.sqrt(dx * dx + dy * dy)

    if distance == 0:
        return 0.0, 0.0, True

    movement = speed * dt

    if movement >= distance:
        # Drone reaches target before dt is finished
        actual_time = distance / speed

        drone.x = target_x
        drone.y = target_y

        return distance, actual_time, True

    # Drone does not reach target
    direction_x = dx / distance
    direction_y = dy / distance

    drone.x += direction_x * movement
    drone.y += direction_y * movement

    return movement, dt, False


def calculate_battery_consumption(time, payload):
    FULL_PAYLOAD = 2.5
    FULL_PAYLOAD_ENDURANCE = 25 * 60
    full_payload_rate = 100.0 / FULL_PAYLOAD_ENDURANCE
    empty_payload_rate = 0.7 * full_payload_rate
    payload_ratio = payload / FULL_PAYLOAD
    consumption_rate = empty_payload_rate + payload_ratio * (full_payload_rate - empty_payload_rate)
    return time * consumption_rate

def pixels_to_meters(distance_pixels):
    PIXELS_PER_METER = 10.0
    return distance_pixels / PIXELS_PER_METER

def meters_to_pixels(distance_meters):
    PIXELS_PER_METER = 10.0
    return distance_meters * PIXELS_PER_METER


def apply_battery_degradation(drone):
    drone.cycle_count += 1
    drone.battery_capacity *= 0.9995

    # Battery can never exceed the new degraded capacity
    drone.battery = min(drone.battery, drone.battery_capacity)

FULL_CHARGE_TIME = 40 * 60

def calculate_charge_time(drone):
    missing_fraction = (
        drone.battery_capacity - drone.battery
    ) / drone.battery_capacity
    return FULL_CHARGE_TIME * missing_fraction

def get_free_charging_pad(charging_pads):
    for pad_id, drone_id in charging_pads.items():
        if drone_id is None:
            return pad_id
    return None

def start_charging(drone, charging_pads):
    pad_id = get_free_charging_pad(charging_pads)
    if pad_id is None:
        return False
    charging_pads[pad_id] = drone.id
    drone.charging_pad = pad_id
    drone.status = "CHARGING"
    return True

def update_charging(drone, dt, charging_pads):
    drone.target_battery = min(drone.target_battery, drone.battery_capacity)
    charge_rate = drone.battery_capacity / FULL_CHARGE_TIME
    charged_amount = charge_rate * dt
    drone.battery = min(drone.battery + charged_amount, drone.battery_capacity)
    
    if drone.battery >= drone.target_battery:
        drone.battery = drone.target_battery
        pad_id = drone.charging_pad
        if pad_id is not None:
            charging_pads[pad_id] = None
        drone.charging_pad = None
        drone.status = "IDLE"

def consume_battery(drone, dt):
    battery_used = calculate_battery_consumption(
        dt,
        drone.payload
    )

    drone.battery -= battery_used
    drone.battery = max(0.0, drone.battery)

    drone.cycle_energy += battery_used

    while drone.cycle_energy >= 100.0:
        drone.cycle_energy -= 100.0
        apply_battery_degradation(drone)


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

def calculate_required_battery(drone, package):
    to_package_pixels = math.sqrt((package.x - drone.x)**2 + (package.y - drone.y)**2)
    to_base_pixels = math.sqrt((BASE[0] - package.x)**2 + (BASE[1] - package.y)**2)
    
    to_package_meters = pixels_to_meters(to_package_pixels)
    to_base_meters = pixels_to_meters(to_base_pixels)
    
    delivery_speed = calculate_speed(package.weight)
    return_speed = calculate_speed(0.0)
    
    time_to_package = to_package_meters / delivery_speed
    time_to_base = to_base_meters / return_speed
    
    delivery_battery = calculate_battery_consumption(time_to_package, package.weight)
    return_battery = calculate_battery_consumption(time_to_base, 0.0)
    
    return delivery_battery + return_battery


SAFETY_RESERVE_FRACTION = 0.10
MAX_LOW_BATTERY_WAIT = 600.0
LOW_BATTERY_FRACTION = 0.20


def mission_departure_energy(drone, package):
    # Same real reserve for every policy; do not silently shrink it near capacity.
    return calculate_required_battery(drone, package) + SAFETY_RESERVE_FRACTION * drone.battery_capacity


def is_low_battery_waiting(drone, charging_pads):
    return (drone.status != "FAILED" and drone.current_package is None and drone.payload == 0
            and math.hypot(drone.x - BASE[0], drone.y - BASE[1]) <= 1e-6
            and drone.battery < LOW_BATTERY_FRACTION * drone.battery_capacity
            and drone.id not in charging_pads.values())


def is_mission_feasible(drone, package, sim_time):
    if package.weight > 2.5:
        return False
    remaining_deadline = package.deadline - sim_time
    delivery_time = calculate_delivery_time(drone, package)
    if delivery_time > remaining_deadline:
        return False
    required_battery = mission_departure_energy(drone, package)
    if required_battery > drone.battery:
        return False
    return True

def get_feasible_drones(drones, package, sim_time):
    feasible = []

    for drone in drones.values():
        if drone.status not in ("IDLE", "WAITING_FOR_CHARGE", "CHARGING"):
            continue

        if is_mission_feasible(drone, package, sim_time):
            feasible.append(drone)

    return feasible

def calculate_speed(payload):
    max_speed = 12.0
    max_payload = 2.5
    if payload <= 0:
        return max_speed
    if payload >= max_payload:
        return max_speed * 0.7
    speed_factor = 1.0 - 0.3 * (payload / max_payload)
    return max_speed * speed_factor

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
            if not drone.charging_managed and start_charging(drone, charging_pads):
                return
            return

        else:
            return

def calculate_delivery_time(drone, package):
    # The deadline applies at drop-off; return energy is checked separately.
    distance_pixels = math.hypot(package.x - drone.x, package.y - drone.y)
    return pixels_to_meters(distance_pixels) / calculate_speed(package.weight)


def calculate_mission_time(drone, package):
    to_package_pixels = math.sqrt((package.x - drone.x)**2 + (package.y - drone.y)**2)
    to_base_pixels = math.sqrt((BASE[0] - package.x)**2 + (BASE[1] - package.y)**2)
    
    to_package_meters = pixels_to_meters(to_package_pixels)
    to_base_meters = pixels_to_meters(to_base_pixels)
    
    delivery_speed = calculate_speed(package.weight)
    return_speed = calculate_speed(0.0)
    
    time_to_package = to_package_meters / delivery_speed
    time_to_base = to_base_meters / return_speed
    
    return time_to_package + time_to_base

def calculate_mission_distance(drone, package):
    to_package = math.sqrt((package.x - drone.x)**2 + (package.y - drone.y)**2)
    to_base = math.sqrt((BASE[0] - package.x)**2 + (BASE[1] - package.y)**2)
    return pixels_to_meters(to_package) + pixels_to_meters(to_base)



def calculate_assignment_features(drones, drone_id, package, sim_time):
    drone = drones[drone_id]
    
    # 1. Deadline slack fraction
    delivery_time = calculate_delivery_time(drone, package)
    remaining_deadline = package.deadline - sim_time
    slack = remaining_deadline - delivery_time
    slack_fraction = slack / max(remaining_deadline, 1e-6)
    deadline_cost = 1.0 - slack_fraction
    
    # 2. Energy fraction
    required_energy = calculate_required_battery(drone, package)
    energy_cost = required_energy / max(drone.battery, 1e-6)
    
    # 3. Distance
    distance = calculate_mission_distance(drone, package)
    
    # 4. Charging risk
    remaining_fraction = (drone.battery - required_energy) / max(drone.battery_capacity, 1e-6)
    if remaining_fraction > 0.30:
        charging_risk = 0.0
    elif remaining_fraction > 0.15:
        charging_risk = 0.5
    else:
        charging_risk = 1.0
    
    return {
        "drone_id": drone.id,
        "deadline_cost": deadline_cost,
        "energy_cost": energy_cost,
        "raw_distance": distance,
        "raw_utilization": drone.total_flight_time,
        "charging_risk": charging_risk,
        "slack": slack
    }


def calculate_assignment_score_v2(features, max_distance, avg_flight_time):
    W_DEADLINE = 10.0
    W_ENERGY = 3.0
    W_DISTANCE = 1.0
    W_BALANCE = 2.0
    
    normalized_distance = features["raw_distance"] / max(max_distance, 1e-6)
    balance_cost = features["raw_utilization"] / max(avg_flight_time, 1e-6)
    
    score = (
        W_DEADLINE * features["deadline_cost"]
        + W_ENERGY * features["energy_cost"]
        + W_DISTANCE * normalized_distance
        + W_BALANCE * balance_cost
    )
    
    return {
        "score": score,
        "deadline": W_DEADLINE * features["deadline_cost"],
        "energy": W_ENERGY * features["energy_cost"],
        "distance": W_DISTANCE * normalized_distance,
        "balance": W_BALANCE * balance_cost,
        "charging": 0.0 # Handled structurally now
    }

def calculate_assignment_score(features, max_distance, avg_flight_time):
    W_DEADLINE = 10.0
    W_ENERGY = 3.0
    W_DISTANCE = 1.0
    W_BALANCE = 2.0
    W_CHARGE = 4.0
    
    normalized_distance = features["raw_distance"] / max(max_distance, 1e-6)
    balance_cost = features["raw_utilization"] / max(avg_flight_time, 1e-6)
    
    score = (
        W_DEADLINE * features["deadline_cost"]
        + W_ENERGY * features["energy_cost"]
        + W_DISTANCE * normalized_distance
        + W_BALANCE * balance_cost
        + W_CHARGE * features["charging_risk"]
    )
    
    return {
        "score": score,
        "deadline": W_DEADLINE * features["deadline_cost"],
        "energy": W_ENERGY * features["energy_cost"],
        "distance": W_DISTANCE * normalized_distance,
        "balance": W_BALANCE * balance_cost,
        "charging": W_CHARGE * features["charging_risk"]
    }


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
            elif not any(calculate_required_battery(ideal, package) + SAFETY_RESERVE_FRACTION * d.battery_capacity
                         <= d.battery_capacity for d in drones.values()):
                reason = "Round trip plus safety reserve exceeds every drone's usable capacity"
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
    departure_energy = mission_departure_energy(drone, package)
    if arrival > package.deadline + 1e-9:
        reasons.append("Arrival after deadline")
    if departure_energy > drone.battery + 1e-9:
        reasons.append("Insufficient battery for delivery, return and common safety reserve")
    return {
        "drone_id": drone.id, "status": drone.status, "battery": drone.battery,
        "capacity": drone.battery_capacity, "arrival": arrival,
        "required_energy": energy, "reasons": reasons,
        "departure_energy": departure_energy,
        "charging_basis": drone.charging_basis,
    }


def schedule_packages(drones, packages, sim_time, charging_pads, algorithm, timestep=1.0):
    update_package_outcomes(drones, packages, sim_time)
    for drone in drones.values():
        drone.charging_managed = algorithm == "v2"
        if is_low_battery_waiting(drone, charging_pads):
            if drone.low_battery_wait_since is None:
                drone.low_battery_wait_since = sim_time
        else:
            drone.low_battery_wait_since = None
    preparation = {}
    if algorithm == "v2":
        preparation = manage_charging_infrastructure(drones, packages, charging_pads, sim_time, timestep)
    package_plans = {plan["package_id"]: plan for plan in preparation.values()}
    ready_at_base = charging_candidates_at_base(drones)
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
                if candidate["drone_id"] not in ready_at_base:
                    candidate["reasons"].append("Drone not physically ready at pickup base")
                drone = drones[candidate["drone_id"]]
                if (drone.recovery_charging and drone.recovery_package is None
                        and drone.battery < LOW_BATTERY_FRACTION * drone.battery_capacity):
                    candidate["reasons"].append("Complete starvation-recovery charge before dispatch")
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
                reason = ("Fleet preparation ready; coordinated coverage and stable pairing"
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
            drone.preparation_package = None
            drone.recovery_charging = False
            drone.recovery_package = None


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


def assign_packages_baseline(drones, packages, sim_time, charging_pads, timestep=1.0):
    schedule_packages(drones, packages, sim_time, charging_pads, "baseline", timestep)


def assign_packages_v1(drones, packages, sim_time, charging_pads, timestep=1.0):
    schedule_packages(drones, packages, sim_time, charging_pads, "v1", timestep)

def charging_candidates_at_base(drones):
    """Only empty, available drones physically at the pickup base can prepare."""
    return {
        d.id: d for d in drones.values()
        if d.status in ("IDLE", "WAITING_FOR_CHARGE", "CHARGING")
        and d.current_package is None and d.payload == 0
        and all(math.isfinite(v) for v in (d.x, d.y, d.battery, d.battery_capacity))
        and d.battery_capacity > 0 and 0 <= d.battery <= d.battery_capacity
        and math.hypot(d.x - BASE[0], d.y - BASE[1]) <= 1e-6
    }


PREPARATION_BEAM_WIDTH = 6
PREPARATION_SWITCH_PENALTY = 5.0  # Simulated arrival seconds per changed pairing.
PREPARATION_JOBS_PER_DRONE = 3


def planning_tick(value, now, timestep):
    return now + max(0, math.ceil((value - now) / timestep - 1e-9)) * timestep


def predict_drone_availability(drone, packages, sim_time, timestep=1.0):
    """Read-only forecast of an empty drone at base after its current mission."""
    values = (drone.x, drone.y, drone.battery, drone.battery_capacity,
              drone.payload, drone.cycle_energy)
    if not all(math.isfinite(v) for v in values):
        return None
    if not (0 <= drone.battery <= drone.battery_capacity and drone.battery_capacity > 0
            and 0 <= drone.payload <= 2.5 and 0 <= drone.cycle_energy < 100):
        return None
    predicted = replace(drone)
    legs = []
    if drone.status in ("IDLE", "CHARGING", "WAITING_FOR_CHARGE"):
        if drone.id not in charging_candidates_at_base({drone.id: drone}):
            return None
        # Charging targets are preemptible; only energy already stored is available now.
    elif drone.status == "DELIVERY":
        package = packages.get(drone.current_package)
        if (package is None or package.delivered or package.assigned_drone != drone.id
                or drone.target is None
                or not all(math.isfinite(v) for v in drone.target)
                or math.hypot(drone.target[0] - package.x, drone.target[1] - package.y) > 1e-6):
            return None
        legs = [(math.hypot(drone.target[0]-drone.x, drone.target[1]-drone.y), drone.payload),
                (math.hypot(BASE[0]-package.x, BASE[1]-package.y), 0.0)]
    elif drone.status == "RETURNING":
        if drone.target is None or tuple(drone.target) != tuple(BASE) or drone.payload != 0:
            return None
        legs = [(math.hypot(BASE[0]-drone.x, BASE[1]-drone.y), 0.0)]
    else:
        return None

    elapsed = 0.0
    for distance, payload in legs:
        duration = pixels_to_meters(distance) / calculate_speed(payload)
        predicted.payload = payload
        old_capacity = predicted.battery_capacity
        consume_battery(predicted, duration)
        # Whole-leg forecasting can miss a capacity clamp at an earlier tick.
        # Subtract the capacity loss as a conservative bound on that discrepancy.
        predicted.battery = max(0.0, predicted.battery - (old_capacity - predicted.battery_capacity))
        if predicted.battery <= 0:
            return None
        elapsed += duration
    predicted.x, predicted.y = float(BASE[0]), float(BASE[1])
    predicted.payload = 0.0
    predicted.current_package = None
    predicted.target = None
    predicted.status = "IDLE"
    predicted.charging_pad = None
    return {
        "state": predicted, "available_at": planning_tick(sim_time + elapsed, sim_time, timestep),
        "physical_arrival": sim_time + elapsed, "source_status": drone.status,
    }


def earliest_pad_slot(calendar, available_at, duration, now, timestep, preferred=None):
    choices = []
    for pad, intervals in calendar.items():
        start = available_at
        for busy_start, busy_end in sorted(intervals):
            end = planning_tick(start + duration, now, timestep)
            if end <= busy_start + 1e-9:
                break
            if start < busy_end:
                start = busy_end
        if math.isfinite(start):
            end = planning_tick(start + duration, now, timestep)
            choices.append((end, pad != preferred, pad, start))
    if not choices:
        return None
    end, _, pad, start = min(choices)
    return pad, start, end


def valid_pending_packages(packages, sim_time):
    return sorted(
        (p for p in packages.values()
         if p.status == "PENDING" and p.assigned_drone is None and not p.delivered
         and all(math.isfinite(v) for v in (p.x, p.y, p.weight, p.request_time, p.deadline))
         and 0 <= p.weight <= 2.5 and p.request_time <= sim_time <= p.deadline),
        key=lambda p: (p.deadline, p.request_time, p.id))


def starvation_recovery_schedule(drones, charging_pads, sim_time, timestep, packages=None):
    eligible = charging_candidates_at_base(drones)
    overdue = sorted(
        (d for d in eligible.values()
         if (d.recovery_charging and d.recovery_package is not None)
         or (d.battery < LOW_BATTERY_FRACTION*d.battery_capacity
             and (d.recovery_charging or (d.low_battery_wait_since is not None
                  and sim_time-d.low_battery_wait_since >= MAX_LOW_BATTERY_WAIT)))),
        key=lambda d: (not d.recovery_charging,
                       d.low_battery_wait_since if d.low_battery_wait_since is not None else sim_time,
                       d.id))
    free = [pad for pad, owner in charging_pads.items() if owner is None or owner in eligible]
    pending = valid_pending_packages(packages or {}, sim_time)
    recovery, claimed = {}, set()
    for drone in overdue:
        options = []
        for package in pending:
            if package.id in claimed:
                continue
            target = mission_departure_energy(drone, package)
            if target > drone.battery_capacity:
                continue
            duration = max(0.0, target-drone.battery)/drone.battery_capacity*FULL_CHARGE_TIME
            if duration > 1e-9 and not free:
                continue
            ready = planning_tick(sim_time+duration, sim_time, timestep)
            arrival = ready+calculate_delivery_time(drone, package)
            if arrival <= package.deadline+1e-9:
                options.append(dict(package_id=package.id, target_battery=target,
                                    charge_seconds=duration, charge_end=ready,
                                    predicted_arrival=arrival, basis="delivery"))
        if options:
            selected = min(options, key=lambda c: (
                c["package_id"] != drone.recovery_package,
                c["predicted_arrival"], c["charge_seconds"], c["package_id"]))
            claimed.add(selected["package_id"])
        else:
            target = LOW_BATTERY_FRACTION*drone.battery_capacity
            if drone.battery >= target or not free:
                continue
            duration = (target-drone.battery)/drone.battery_capacity*FULL_CHARGE_TIME
            selected = dict(package_id=None, target_battery=target, charge_seconds=duration,
                            charge_end=planning_tick(sim_time+duration, sim_time, timestep),
                            basis="fallback_20_percent")
        pad = None
        if selected["charge_seconds"] > 1e-9:
            pad = drone.charging_pad if drone.charging_pad in free else free[0]
            free.remove(pad)
        recovery[drone.id] = dict(selected, pad_id=pad, charge_start=sim_time)
    return recovery


def forecast_after_planned_job(prediction, candidate, package, sim_time, timestep):
    state = replace(prediction["state"])
    state.battery = max(state.battery, candidate["target_battery"])
    outbound = calculate_delivery_time(state, package)
    returning = calculate_mission_time(state, package)-outbound
    for duration, payload in ((outbound, package.weight), (returning, 0.0)):
        state.payload = payload
        old_capacity = state.battery_capacity
        consume_battery(state, duration)
        state.battery = max(0.0, state.battery-(old_capacity-state.battery_capacity))
        state.total_flight_time += duration
    if state.battery <= 0:
        return None
    physical_arrival = candidate["charge_end"]+outbound+returning
    return dict(state=state, available_at=planning_tick(physical_arrival, sim_time, timestep),
                physical_arrival=physical_arrival, source_status="PLANNED_RETURN")


def job_candidate(drone_id, prediction, package, calendar, drones, sim_time, timestep, fixed=None):
    drone = prediction["state"]
    target = mission_departure_energy(drone, package)
    if target > drone.battery_capacity:
        return None
    added = max(0.0, target-drone.battery)
    duration = added/drone.battery_capacity*FULL_CHARGE_TIME
    pad, start, ready = None, prediction["available_at"], prediction["available_at"]
    if fixed is not None:
        pad, start, ready = fixed["pad_id"], fixed["charge_start"], fixed["charge_end"]
    elif duration > 1e-9:
        slot = earliest_pad_slot(calendar, start, duration, sim_time, timestep, drones[drone_id].charging_pad)
        if slot is None:
            return None
        pad, start, ready = slot
    arrival = ready+calculate_delivery_time(drone, package)
    if arrival > package.deadline+1e-9:
        return None
    return dict(drone_id=drone_id, package_id=package.id,
                available_at=prediction["available_at"], physical_arrival=prediction["physical_arrival"],
                source_status=prediction["source_status"], battery_at_availability=drone.battery,
                capacity_at_availability=drone.battery_capacity,
                required_energy=calculate_required_battery(drone, package), target_battery=target,
                added_energy=added, charge_seconds=duration, mission_seconds=calculate_mission_time(drone, package),
                pad_id=pad, charge_start=start, charge_end=ready, predicted_arrival=arrival,
                slack=package.deadline-arrival)


def select_diverse_beam(expanded, rank, best_skip, sim_time, width=PREPARATION_BEAM_WIDTH):
    def exact(branch):
        return tuple(sorted((d, tuple((p["package_id"], p["pad_id"], p["charge_start"], p["charge_end"])
                                      for p in route)) for d, route in branch["routes"].items()))
    def strategy(branch):
        # Ignore symmetric drone/pad labels; preserve route orders and materially
        # different charging requirements or start times (one-minute buckets).
        return tuple(sorted((tuple(p["package_id"] for p in route),
                             tuple((int(p["charge_seconds"]//60),
                                    int((p["charge_start"]-sim_time)//60)) for p in route))
                            for route in branch["routes"].values()))
    unique = {}
    for branch in sorted(expanded, key=rank):
        unique.setdefault(exact(branch), branch)
    ordered = list(unique.values())
    selected = [ordered[0]]
    if exact(best_skip) != exact(selected[0]) and width > 1:
        selected.append(best_skip)
    seen = {strategy(branch) for branch in selected}
    best = rank(ordered[0])
    for branch in ordered:
        if len(selected) >= width:
            break
        score = rank(branch)
        competitive = score[0] == best[0] and score[1] <= best[1]+max(10.0, .25*abs(best[1]))
        if competitive and strategy(branch) not in seen:
            selected.append(branch)
            seen.add(strategy(branch))
    chosen = {exact(branch) for branch in selected}
    for branch in ordered:
        if len(selected) >= width:
            break
        if exact(branch) not in chosen:
            selected.append(branch)
            chosen.add(exact(branch))
    return selected


def plan_fleet_preparation(drones, packages, charging_pads, sim_time, timestep=1.0):
    if not math.isfinite(timestep) or timestep <= 0:
        raise ValueError("Planning timestep must be finite and positive")
    future = {d.id: prediction for d in drones.values()
              if (prediction := predict_drone_availability(d, packages, sim_time, timestep)) is not None}
    at_base = charging_candidates_at_base(drones)
    base_calendar = {pad: ([] if owner is None or owner in at_base else [(sim_time, float("inf"))])
                     for pad, owner in charging_pads.items()}
    recovery = starvation_recovery_schedule(drones, charging_pads, sim_time, timestep, packages)
    seed_routes, pinned_packages = {}, set()
    for drone_id, session in recovery.items():
        if session["package_id"] is not None:
            package = packages[session["package_id"]]
            job = job_candidate(drone_id, future[drone_id], package, base_calendar,
                                drones, sim_time, timestep, fixed=session)
            if job is None:
                continue
            job["recovery_delivery"] = True
            seed_routes[drone_id] = [job]
            pinned_packages.add(package.id)
            future[drone_id] = forecast_after_planned_job(future[drone_id], job, package, sim_time, timestep)
        else:
            state = replace(future[drone_id]["state"], battery=session["target_battery"])
            future[drone_id] = dict(state=state, available_at=session["charge_end"],
                                   physical_arrival=sim_time, source_status="RECOVERY_CHARGE")
        if session["pad_id"] is not None:
            base_calendar[session["pad_id"]].append((session["charge_start"],session["charge_end"]))
    future = {d:p for d,p in future.items() if p is not None}
    pending = [p for p in valid_pending_packages(packages, sim_time) if p.id not in pinned_packages]
    previous = {d.preparation_package:d.id for d in drones.values() if d.preparation_package is not None}

    def rank(branch):
        routes = branch["routes"]
        jobs = [p for route in routes.values() for p in route]
        resources = sum(p["charge_seconds"]/max(len(charging_pads),1)
                        + p["mission_seconds"]/max(len(drones),1) for p in jobs)
        switches = sum(PREPARATION_SWITCH_PENALTY for d,route in routes.items()
                       if route[0]["package_id"] in previous and previous[route[0]["package_id"]] != d)
        return (-len(jobs), resources+.05*(switches+sum(p["predicted_arrival"]-sim_time for p in jobs)),
                sum(packages[p["package_id"]].deadline for p in jobs),
                sum(drones[d].total_flight_time for d in routes),
                tuple(sorted((d,tuple(p["package_id"] for p in route)) for d,route in routes.items())))

    def replay(drone_id, order, routes):
        calendar = {pad:list(intervals) for pad,intervals in base_calendar.items()}
        for other, route in routes.items():
            if other == drone_id:
                continue
            for job in route[len(seed_routes.get(other,[])):]:
                if job["pad_id"] is not None:
                    calendar[job["pad_id"]].append((job["charge_start"],job["charge_end"]))
        rebuilt = list(seed_routes.get(drone_id,[]))
        prediction = future[drone_id]
        for package_id in order:
            package = packages[package_id]
            job = job_candidate(drone_id,prediction,package,calendar,drones,sim_time,timestep)
            if job is None:
                return None
            prediction = forecast_after_planned_job(prediction,job,package,sim_time,timestep)
            if prediction is None:
                return None
            rebuilt.append(job)
            if job["pad_id"] is not None:
                calendar[job["pad_id"]].append((job["charge_start"],job["charge_end"]))
        new_routes = dict(routes)
        new_routes[drone_id] = rebuilt
        return dict(routes=new_routes)

    beam = [dict(routes=seed_routes)]
    for package in pending:
        expanded = list(beam)
        for branch in beam:
            for drone_id in future:
                route = branch["routes"].get(drone_id,[])
                if len(route) >= PREPARATION_JOBS_PER_DRONE:
                    continue
                prefix = len(seed_routes.get(drone_id,[]))
                suffix = [p["package_id"] for p in route[prefix:]]
                # Insert at every position, replaying all affected missions and
                # pad reservations. A three-job route can explore all six orders.
                for position in range(len(suffix)+1):
                    order = suffix[:position]+[package.id]+suffix[position:]
                    trial = replay(drone_id,order,branch["routes"])
                    if trial is not None:
                        expanded.append(trial)
        beam = select_diverse_beam(expanded,rank,min(beam,key=rank),sim_time)
    winner = min(beam,key=rank)
    plans = {}
    for drone_id,route in winner["routes"].items():
        alternatives = [dict(job) for branch in beam for jobs in branch["routes"].values()
                        for job in jobs[:1] if job["package_id"] == route[0]["package_id"]]
        plans[drone_id] = dict(route[0], sequence=[dict(job) for job in route], alternatives=alternatives,
                              horizon_deliveries=sum(len(r) for r in winner["routes"].values()))
    return plans


def opportunistic_readiness(drone, packages, covered, sim_time, timestep=1.0):
    choices = []
    for package in valid_pending_packages(packages, sim_time):
        if package.id in covered:
            continue
        target = mission_departure_energy(drone,package)
        if target > drone.battery_capacity:
            continue
        seconds = max(0.0,target-drone.battery)/drone.battery_capacity*FULL_CHARGE_TIME
        if planning_tick(sim_time+seconds,sim_time,timestep)+calculate_delivery_time(drone,package) <= package.deadline+1e-9:
            choices.append(dict(target_battery=target, seconds_to_ready=seconds,
                                kind="uncovered_pending", package_id=package.id))
    if choices:
        return min(choices,key=lambda c:(c["seconds_to_ready"],c["package_id"]))
    # Forecast only from already observed requests, never from future arrivals.
    observed = sorted(
        (p for p in packages.values() if p.request_time <= sim_time
         and all(math.isfinite(v) for v in (p.x,p.y,p.weight,p.request_time))
         and 0 <= p.weight <= 2.5),
        key=lambda p:(p.request_time,p.id), reverse=True)[:50]
    targets = [mission_departure_energy(drone,p) for p in observed
               if mission_departure_energy(drone,p) <= drone.battery_capacity]
    target = statistics.median(targets) if targets else LOW_BATTERY_FRACTION*drone.battery_capacity
    return dict(target_battery=target,
                seconds_to_ready=max(0.0,target-drone.battery)/drone.battery_capacity*FULL_CHARGE_TIME,
                kind="recent_request_estimate" if targets else "fallback_20_percent",
                sample_size=len(targets))


def manage_charging_infrastructure(drones, packages, charging_pads, sim_time, timestep=1.0):
    plans = plan_fleet_preparation(drones, packages, charging_pads, sim_time, timestep)
    recovery = starvation_recovery_schedule(drones, charging_pads, sim_time, timestep, packages)
    eligible = charging_candidates_at_base(drones)
    for drone in drones.values():
        drone.charging_managed = True
        drone.recovery_charging = drone.id in recovery
        drone.recovery_package = recovery[drone.id]["package_id"] if drone.id in recovery else None
        drone.preparation_package = plans[drone.id]["package_id"] if drone.id in plans else None
        drone.charging_basis = None
    new_pads = {
        pad: (owner if owner is not None and owner not in eligible else None)
        for pad, owner in charging_pads.items()
    }
    for drone in eligible.values():
        drone.target_battery = (recovery[drone.id]["target_battery"] if drone.id in recovery
                               else plans[drone.id]["target_battery"] if drone.id in plans
                               else drone.battery_capacity)
        if drone.id in recovery:
            drone.charging_basis = dict(kind="starvation_recovery", **recovery[drone.id])
        elif drone.id in plans:
            drone.charging_basis = dict(kind="planned_delivery", package_id=plans[drone.id]["package_id"],
                                       target_battery=drone.target_battery)
    for drone_id, session in recovery.items():
        if session["pad_id"] is not None:
            new_pads[session["pad_id"]] = drone_id
    # Future reservation is metadata only: never move an airborne drone onto a pad.
    for drone_id, plan in plans.items():
        if (drone_id in eligible and drone_id not in recovery and plan["pad_id"] is not None
                and plan["charge_start"] <= sim_time + 1e-9):
            new_pads[plan["pad_id"]] = drone_id
    covered = {job["package_id"] for plan in plans.values() for job in plan["sequence"]}
    opportunistic = [d for d in eligible.values()
                     if d.id not in plans and d.id not in recovery and d.battery < d.battery_capacity]
    while opportunistic:
        free = [pad for pad, owner in new_pads.items() if owner is None]
        if not free:
            break
        choices = [(d, opportunistic_readiness(d, packages, covered, sim_time, timestep))
                   for d in opportunistic]
        # Complete a useful readiness threshold before topping up ready drones.
        drone, readiness = min(choices, key=lambda item: (
            item[1]["seconds_to_ready"] <= 1e-9, item[1]["seconds_to_ready"],
            item[0].charging_pad is None, item[0].id))
        opportunistic.remove(drone)
        pad = drone.charging_pad if drone.charging_pad in free else free[0]
        new_pads[pad] = drone.id
        if readiness["kind"] == "uncovered_pending":
            covered.add(readiness["package_id"])
        drone.target_battery = (readiness["target_battery"] if readiness["seconds_to_ready"] > 1e-9
                                else drone.battery_capacity)
        drone.charging_basis = dict(readiness, kind=(readiness["kind"] if readiness["seconds_to_ready"] > 1e-9
                                                   else "ready_drone_top_up"))
        future_starts = [job["charge_start"] for plan in plans.values() for job in plan["sequence"]
                         if job["pad_id"] == pad and job["charge_start"] > sim_time]
        if future_starts:
            # Fill a usable gap, yielding by the reserved start on the simulation clock.
            drone.target_battery = min(drone.target_battery, drone.battery +
                (min(future_starts)-sim_time) * drone.battery_capacity / FULL_CHARGE_TIME)
            drone.charging_basis["yield_by"] = min(future_starts)
        drone.charging_basis["allocated_target"] = drone.target_battery
    allocated = {owner: pad for pad, owner in new_pads.items() if owner is not None}
    charging_pads.update(new_pads)
    for drone in eligible.values():
        drone.charging_pad = allocated.get(drone.id)
        if drone.charging_pad is not None:
            drone.status = "CHARGING"
        elif (drone.id in plans or drone.id in recovery) and drone.battery + 1e-9 < drone.target_battery:
            drone.status = "WAITING_FOR_CHARGE"
        else:
            drone.status = "IDLE"
    return plans

def assign_packages_v2(drones, packages, sim_time, charging_pads, timestep=1.0):
    schedule_packages(drones, packages, sim_time, charging_pads, "v2", timestep)

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
                scheduler(drones, packages, sim_time, charging_pads, timestep=dt)
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

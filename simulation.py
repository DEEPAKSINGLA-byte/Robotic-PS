import math
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

def calculate_energy_consumption(distance, payload):
    base_consumption = 100.0 / 2000.0
    payload_factor = 1.0 + 0.5 * (payload / 2.5)
    return distance * base_consumption * payload_factor

def calculate_battery_consumption(time, payload):
    FULL_PAYLOAD = 2.5
    FULL_PAYLOAD_ENDURANCE = 2.5 * 60
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

def calculate_travel_time(distance_pixels, payload):
    distance_meters = pixels_to_meters(distance_pixels)
    speed = calculate_speed(payload)
    if speed <= 0:
        return float("inf")
    return distance_meters / speed

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
    charge_rate = drone.battery_capacity / FULL_CHARGE_TIME
    charged_amount = charge_rate * dt
    drone.battery += charged_amount
    
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

def should_create_package(sim_time):
    # Deterministic arrivals every 20 seconds
    return sim_time > 0 and (sim_time % 20.0) < 0.001

def generate_package(package_id, sim_time):
    dx = [100, -100, 200, -200, 300, -300, 150, -150, 250, -250]
    dy = [100, 150, -100, -150, 200, 250, -200, -250, 50, -50]
    idx = package_id % 10
    x = BASE[0] + dx[idx]
    y = BASE[1] + dy[idx]
    weight = 1.0 + (package_id % 3) * 0.5
    deadline = sim_time + 100.0
    
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

def calculate_battery_margin(drone, package):
    required = calculate_required_battery(drone, package)
    return drone.battery - required

def is_mission_feasible(drone, package, sim_time):
    if package.weight > 2.5:
        return False
    remaining_deadline = package.deadline - sim_time
    mission_time = calculate_mission_time(drone, package)
    if mission_time > remaining_deadline:
        return False
    required_battery = calculate_required_battery(drone, package)
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

def update_drone(drone, packages, dt, charging_pads):

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

def estimate_expected_wait_time(drones, packages, charging_pads, sim_time, candidate_drone_id, candidate_eta, candidate_battery_after):
    # 1. Initialize pad free times
    pad_free_times = []
    for occupant_id in charging_pads.values():
        if occupant_id is not None:
            occupant = drones[occupant_id]
            pad_free_times.append(sim_time + occupant.charge_remaining)
        else:
            pad_free_times.append(sim_time)
            
    # 2. Compile future queue
    future_events = []
    for drone in drones.values():
        if drone.id == candidate_drone_id:
            continue
            
        if drone.status == "WAITING_FOR_CHARGE":
            eta = sim_time
            charge_time = calculate_charge_time(drone)
            future_events.append((eta, charge_time))
            
        elif drone.status in ("DELIVERY", "RETURNING"):
            if drone.status == "DELIVERY":
                package = packages[drone.current_package]
                dist_to_pkg = math.sqrt((package.x - drone.x)**2 + (package.y - drone.y)**2)
                dist_to_base = math.sqrt((BASE[0] - package.x)**2 + (BASE[1] - package.y)**2)
                dist = dist_to_pkg + dist_to_base
            else:
                dist = math.sqrt((BASE[0] - drone.x)**2 + (BASE[1] - drone.y)**2)
                
            speed = calculate_speed(drone.payload)
            flight_time = pixels_to_meters(dist) / speed
            eta = sim_time + flight_time
            
            battery_used = calculate_battery_consumption(flight_time, drone.payload)
            bat_after = drone.battery - battery_used
            
            if bat_after < 0.20 * drone.battery_capacity:
                missing_frac = (drone.battery_capacity - bat_after) / max(drone.battery_capacity, 1e-6)
                charge_time = FULL_CHARGE_TIME * missing_frac
                future_events.append((eta, charge_time))
                
    # 3. Simulate queue
    future_events.sort(key=lambda x: x[0])
    for eta, charge_time in future_events:
        pad_free_times.sort()
        start_charge = max(eta, pad_free_times[0])
        pad_free_times[0] = start_charge + charge_time
        
    # 4. Evaluate candidate
    if candidate_battery_after < 0.20 * drones[candidate_drone_id].battery_capacity:
        pad_free_times.sort()
        start_charge = max(candidate_eta, pad_free_times[0])
        return start_charge - candidate_eta
    
    return 0.0

def calculate_assignment_features(drones, drone_id, package, sim_time):
    drone = drones[drone_id]
    
    # 1. Deadline slack fraction
    mission_time = calculate_mission_time(drone, package)
    remaining_deadline = package.deadline - sim_time
    slack = remaining_deadline - mission_time
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

def calculate_assignment_features_v2(drones, packages, charging_pads, drone_id, package, sim_time):
    drone = drones[drone_id]
    
    # 1. Deadline slack fraction
    mission_time = calculate_mission_time(drone, package)
    remaining_deadline = package.deadline - sim_time
    slack = remaining_deadline - mission_time
    slack_fraction = slack / max(remaining_deadline, 1e-6)
    deadline_cost = 1.0 - slack_fraction
    
    # 2. Energy fraction
    required_energy = calculate_required_battery(drone, package)
    energy_cost = required_energy / max(drone.battery, 1e-6)
    
    # 3. Distance
    distance = calculate_mission_distance(drone, package)
    
    # 4. Expected Charging Wait
    battery_after = drone.battery - required_energy
    eta = sim_time + mission_time
    expected_wait = estimate_expected_wait_time(
        drones, packages, charging_pads, sim_time, drone_id, eta, battery_after
    )
    
    return {
        "drone_id": drone.id,
        "deadline_cost": deadline_cost,
        "energy_cost": energy_cost,
        "raw_distance": distance,
        "raw_utilization": drone.total_flight_time,
        "expected_wait": expected_wait,
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

def get_next_package(packages):
    for package in packages.values():
        if not package.delivered and package.assigned_drone is None:
            return package

    return None

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

def assign_packages_baseline(drones, packages, sim_time, charging_pads):
    for package in packages.values():
        if package.delivered:
            continue
        if package.assigned_drone is not None:
            continue

        drone = get_idle_drone(drones)
        if drone is None:
            continue

        if is_mission_feasible(drone, package, sim_time):
            assign_package(drone, package)
        else:
            package.assigned_drone = -1

def assign_packages_v1(drones, packages, sim_time, charging_pads):
    for package in packages.values():
        if package.delivered:
            continue
        if package.assigned_drone is not None:
            continue

        feasible_drones = get_feasible_drones(drones, package, sim_time)
        
        if not feasible_drones:
            # We don't mark as impossible yet unless strictly needed, but baseline did.
            # Actually, baseline only marked if the single chosen drone was infeasible.
            # If no drones are feasible right now, they might be charging.
            # So we leave it unassigned to retry later.
            continue
            
        features_list = []
        for drone in feasible_drones:
            features_list.append(calculate_assignment_features(drones, drone.id, package, sim_time))
            
        max_distance = max((f["raw_distance"] for f in features_list), default=1e-6)
        avg_flight_time = sum(d.total_flight_time for d in drones.values()) / len(drones)
        
        best_drone = None
        best_score = float('inf')
        best_details = None
        all_scores = {}
        
        for f in features_list:
            score_details = calculate_assignment_score(f, max_distance, avg_flight_time)
            all_scores[f["drone_id"]] = score_details
            if score_details["score"] < best_score:
                best_score = score_details["score"]
                best_drone = drones[f["drone_id"]]
                best_details = score_details
                
        if best_drone is not None:
            assign_package(best_drone, package)
            if best_drone.charging_pad is not None:
                charging_pads[best_drone.charging_pad] = None
                best_drone.charging_pad = None

def predict_mission_needs(drone, packages, sim_time):
    best_package = None
    best_cost = float('inf')
    best_slack = float('inf')
    
    # Simple prediction cost: distance + deadline
    for package in packages.values():
        if package.delivered or package.assigned_drone is not None:
            continue
        
        # Must be feasible
        if not is_mission_feasible(drone, package, sim_time):
            continue
            
        mission_time = calculate_mission_time(drone, package)
        remaining_deadline = package.deadline - sim_time
        slack = remaining_deadline - mission_time
        slack_fraction = slack / max(remaining_deadline, 1e-6)
        deadline_cost = 1.0 - slack_fraction
        
        dist = calculate_mission_distance(drone, package)
        cost = deadline_cost + (dist / 10000.0)
        
        if cost < best_cost:
            best_cost = cost
            best_package = package
            best_slack = slack
            
    if best_package is None:
        return 0.0, float('inf')
        
    req_energy = calculate_required_battery(drone, best_package)
    return req_energy, best_slack

def manage_charging_infrastructure(drones, packages, charging_pads, sim_time):
    essential_queue = []
    opportunistic_queue = []
    
    # 1. Categorize Demand
    for drone in drones.values():
        if drone.status not in ("IDLE", "WAITING_FOR_CHARGE", "CHARGING"):
            continue
            
        req_energy, predicted_slack = predict_mission_needs(drone, packages, sim_time)
        safety_margin = 0.10 * drone.battery_capacity
        essential_target = req_energy + safety_margin
        
        drone.target_battery = 100.0 # Default opportunistic target
        
        if drone.battery < essential_target:
            # Does it need charging for an urgent mission?
            if predicted_slack <= 60.0:
                drone.target_battery = essential_target
                essential_queue.append((drone, predicted_slack))
            else:
                opportunistic_queue.append((drone, predicted_slack))
        elif drone.battery < drone.battery_capacity:
            opportunistic_queue.append((drone, predicted_slack))
            
    # Sort queues by urgency (smallest slack first)
    essential_queue.sort(key=lambda x: x[1])
    opportunistic_queue.sort(key=lambda x: x[1])
    
    # Combine queues for allocation priority
    priority_list = [d for d, s in essential_queue] + [d for d, s in opportunistic_queue]
    
    # 2. Re-evaluate Pad Priorities (Preempt opportunistic if essential needs pad)
    # We figure out which drones *should* get pads
    assigned_pads = {}
    remaining_pads = list(charging_pads.keys())
    
    # Keep currently charging drones on their pads if they are high enough priority
    # This prevents physically swapping pads unnecessarily
    allocated_drones = set()
    
    for drone in priority_list:
        if not remaining_pads:
            break
            
        # If it was already charging, try to keep it on the same pad
        if drone.status == "CHARGING" and drone.charging_pad in remaining_pads:
            pad = drone.charging_pad
            remaining_pads.remove(pad)
        else:
            pad = remaining_pads.pop(0)
            
        assigned_pads[pad] = drone.id
        allocated_drones.add(drone.id)
        
    # 3. Apply state changes
    for pad_id in charging_pads.keys():
        new_drone_id = assigned_pads.get(pad_id)
        old_drone_id = charging_pads[pad_id]
        
        if old_drone_id != new_drone_id:
            # Kick old drone off
            if old_drone_id is not None:
                old_drone = drones[old_drone_id]
                old_drone.charging_pad = None
                # If it's essential, it goes to WAITING, else IDLE
                is_essential = any(d.id == old_drone_id for d, _ in essential_queue)
                old_drone.status = "WAITING_FOR_CHARGE" if is_essential else "IDLE"
                
            # Put new drone on
            if new_drone_id is not None:
                new_drone = drones[new_drone_id]
                new_drone.charging_pad = pad_id
                new_drone.status = "CHARGING"
                
        charging_pads[pad_id] = new_drone_id
        
    # Any drone not allocated a pad but in essential queue goes to WAITING_FOR_CHARGE
    for drone, slack in essential_queue:
        if drone.id not in allocated_drones:
            drone.status = "WAITING_FOR_CHARGE"
            
    # Any drone not allocated a pad in opportunistic queue goes to IDLE
    for drone, slack in opportunistic_queue:
        if drone.id not in allocated_drones:
            drone.status = "IDLE"

def assign_packages_v2(drones, packages, sim_time, charging_pads):
    # Stages 1-3: Actively manage charging queue based on predictive partial charging
    manage_charging_infrastructure(drones, packages, charging_pads, sim_time)

    # Stage 4: Assignment
    for package in packages.values():
        if package.delivered:
            continue
        if package.assigned_drone is not None:
            continue

        feasible_drones = get_feasible_drones(drones, package, sim_time)
        
        if not feasible_drones:
            continue
            
        features_list = []
        for drone in feasible_drones:
            features_list.append(calculate_assignment_features(drones, drone.id, package, sim_time))
            
        max_distance = max((f["raw_distance"] for f in features_list), default=1e-6)
        avg_flight_time = sum(d.total_flight_time for d in drones.values()) / len(drones)
        
        best_drone = None
        best_score = float('inf')
        best_details = None
        all_scores = {}
        
        for f in features_list:
            score_details = calculate_assignment_score_v2(f, max_distance, avg_flight_time)
            all_scores[f["drone_id"]] = score_details
            if score_details["score"] < best_score:
                best_score = score_details["score"]
                best_drone = drones[f["drone_id"]]
                best_details = score_details
                
        if best_drone is not None:
            assign_package(best_drone, package)
            if best_drone.charging_pad is not None:
                charging_pads[best_drone.charging_pad] = None
                best_drone.charging_pad = None

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
    update_drone(drone, packages, dt, charging_pads)

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

def main():
    drones = initialize_drones()
    packages = create_test_packages()
    charging_pads = {1: None, 2: None, 3: None}
    
    env = Drone2DEnvironment(num_drones=4)
    cv2.namedWindow("Drone 2D Environment", cv2.WINDOW_NORMAL)
    
    dt_real = 0.1
    TIME_SCALE = 2.5
    sim_time = 0.0
    
    while True:
        dt_sim = dt_real * TIME_SCALE
        
        for drone in drones.values():
            run_simulation_step(drone, packages, dt_sim, sim_time, charging_pads)
        
        validate_simulation(drones, charging_pads)
        
        sim_time += dt_sim
        
        # Determine pad_occupancy array for UI visualization
        pad_occupancy_list = [charging_pads[1], charging_pads[2], charging_pads[3]]
        
        env.update_state(
            drones=drones,
            packages=packages,
            pad_occupancy=pad_occupancy_list,
            message=f"Sim Time: {sim_time:.1f}s"
        )
        
        frame = env.draw()
        cv2.imshow("Drone 2D Environment", frame)
        key = cv2.waitKey(int(dt_real * 1000)) & 0xFF
        
        if all(package.delivered or package.assigned_drone == -1 for package in packages.values()):
            print("\nALL FEASIBLE PACKAGES DELIVERED (OR REJECTED)")
            break
            
        if any(drone.status == "FAILED" for drone in drones.values()):
            print("\nDRONE FAILED")
            break
            
        if key in (ord('q'), ord('Q'), 27):
            break
            
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
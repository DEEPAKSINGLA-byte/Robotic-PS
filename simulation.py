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
    drone.charge_remaining = calculate_charge_time(drone)
    return True

def update_charging(drone, dt, charging_pads):
    drone.charge_remaining -= dt
    if drone.charge_remaining > 0:
        return
    drone.battery = drone.battery_capacity
    pad_id = drone.charging_pad
    if pad_id is not None:
        charging_pads[pad_id] = None
    drone.charging_pad = None
    drone.charge_remaining = 0.0
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

def create_test_packages():
    return {
        1: PackageState(
            id=1,
            x=550.0,
            y=300.0,
            weight=1.0,
            request_time=0.0,
            deadline=60.0,
            assigned_drone=None,
            delivered=False
        ),

        2: PackageState(
            id=2,
            x=650.0,
            y=450.0,
            weight=1.5,
            request_time=0.0,
            deadline=90.0,
            assigned_drone=None,
            delivered=False
        ),

        3: PackageState(
            id=3,
            x=300.0,
            y=500.0,
            weight=2.0,
            request_time=0.0,
            deadline=120.0,
            assigned_drone=None,
            delivered=False
        ),

        4: PackageState(
            id=4,
            x=600.0,
            y=350.0,
            weight=3.0,
            request_time=0.0,
            deadline=60.0,
            assigned_drone=None,
            delivered=False
        ),
    }

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

                if drone.battery < drone.battery_capacity:
                    if not start_charging(drone, charging_pads):
                        drone.status = "WAITING_FOR_CHARGE"
                else:
                    drone.status = "IDLE"

                drone.current_package = None

                continue

        elif drone.status == "CHARGING":
            update_charging(drone, remaining_dt, charging_pads)
            return

        elif drone.status == "WAITING_FOR_CHARGE":
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
    return to_package + to_base

def get_next_package(packages):
    for package in packages.values():
        if not package.delivered and package.assigned_drone is None:
            return package

    return None

def run_simulation_step(drone, packages, dt, sim_time, charging_pads):

    if drone.status == "IDLE":

        package = get_next_package(packages)

        if package is not None:
            if is_mission_feasible(drone, package, sim_time):
                assign_package(drone, package)
            else:
                print(f"Package {package.id} is not feasible. Marking as impossible.")
                package.assigned_drone = -1

    update_drone(drone, packages, dt, charging_pads)

def initialize_drones():
    drones = {}
    batteries = [20.0, 30.0, 40.0, 50.0]
    for i in range(1, 5):
        drones[i] = DroneState(
            id=i, x=float(BASE[0]), y=float(BASE[1]), battery=batteries[i-1],
            status="IDLE", payload=0.0, target=None
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
        
        print("PADS:", charging_pads)
        for drone in drones.values():
            print(
                f"D{drone.id}: "
                f"status={drone.status} "
                f"battery={drone.battery:.2f}/"
                f"{drone.battery_capacity:.2f} "
                f"pad={drone.charging_pad}"
            )
            run_simulation_step(drone, packages, dt_sim, sim_time, charging_pads)
        
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
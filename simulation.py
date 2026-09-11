import math
import cv2
from drone_sim import DroneState, PackageState, BASE, Drone2DEnvironment

def move_towards(drone, target, speed, dt):
    target_x, target_y = target
    dx = target_x - drone.x
    dy = target_y - drone.y
    distance = math.sqrt(dx * dx + dy * dy)
    if distance == 0:
        return 0.0, True
    movement = speed * dt
    if movement >= distance:
        drone.x = target_x
        drone.y = target_y
        return distance, True
    direction_x = dx / distance
    direction_y = dy / distance
    drone.x += direction_x * movement
    drone.y += direction_y * movement
    return movement, False

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

def consume_battery(drone, dt):
    battery_used = calculate_battery_consumption(dt, drone.payload)
    drone.battery -= battery_used
    drone.battery = max(0.0, drone.battery)

def create_test_package():
    return PackageState(
        id=1, x=BASE[0] + 1500, y=BASE[1], weight=2.5, deadline_remaining=2000.0,
        assigned_drone=None, delivered=False
    )

def assign_test_drone(drone, package):
    drone.status = "DELIVERY"
    drone.target = (package.x, package.y)
    drone.payload = package.weight
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

def is_mission_feasible(drone, package):
    if package.weight > 2.5:
        return False
    mission_time = calculate_mission_time(drone, package)
    if mission_time > package.deadline_remaining:
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

def update_drone(drone, package, dt):
    speed_meters = calculate_speed(drone.payload)
    speed_pixels = meters_to_pixels(speed_meters)

    if drone.status == "DELIVERY":
        distance_moved, reached = move_towards(drone, drone.target, speed_pixels, dt)
        consume_battery(drone, dt)
        
        if drone.battery <= 0:
            drone.battery = 0
            drone.status = "FAILED"
            drone.target = None
            return
            
        if reached:
            package.delivered = True
            drone.payload = 0.0
            drone.status = "RETURNING"
            drone.target = (BASE[0], BASE[1])

    elif drone.status == "RETURNING":
        distance_moved, reached = move_towards(drone, drone.target, speed_pixels, dt)
        consume_battery(drone, dt)
        
        if drone.battery <= 0:
            drone.battery = 0
            drone.status = "FAILED"
            drone.target = None
            return
            
        if reached:
            drone.status = "IDLE"
            drone.payload = 0.0
            drone.target = None

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

def run_simulation_step(drones, package, dt):
    for drone in drones.values():
        update_drone(drone, package, dt)

def initialize_drones():
    drones = {}
    drones[1] = DroneState(
        id=1, x=float(BASE[0]), y=float(BASE[1]), battery=100.0,
        status="IDLE", payload=0.0, target=None
    )
    return drones

def main():
    drones = initialize_drones()
    package = create_test_package()
    
    drone = drones[1]
    
    print("Mission time: {} seconds".format(calculate_mission_time(drone, package)))
    print("Required battery: {} %".format(calculate_required_battery(drone, package)))
    print("Available battery: {} %".format(drone.battery))
    print("Feasible: {}".format(is_mission_feasible(drone, package)))
    
    if not is_mission_feasible(drone, package):
        print("Mission is not feasible.")
        return
        
    assign_test_drone(drone, package)
    env = Drone2DEnvironment(num_drones=1)
    cv2.namedWindow("Drone 2D Environment", cv2.WINDOW_NORMAL)
    
    dt_real = 0.1
    TIME_SCALE = 2.5
    sim_time = 0.0
    
    while True:
        dt_sim = dt_real * TIME_SCALE
        drone = drones[1]
        
        print(
            "BEFORE: sim_time={:.1f}s status={:10s} pos=({:.1f}, {:.1f}) battery={:.2f}%".format(
                sim_time, drone.status, drone.x, drone.y, drone.battery
            )
        )
        
        run_simulation_step(drones, package, dt_sim)
        
        sim_time += dt_sim
        
        print(
            "AFTER:  sim_time={:.1f}s status={:10s} pos=({:.1f}, {:.1f}) battery={:.2f}%".format(
                sim_time, drone.status, drone.x, drone.y, drone.battery
            )
        )
        
        env.update_state(
            drones=drones,
            packages={package.id: package},
            message="D1 status: {}".format(drone.status)
        )
        
        frame = env.draw()
        cv2.imshow("Drone 2D Environment", frame)
        key = cv2.waitKey(int(dt_real * 1000)) & 0xFF
        
        if package.delivered and drone.status == "IDLE":
            print("\nMISSION SUCCESS")
            break
            
        if drone.status == "FAILED":
            print("\nMISSION FAILED: battery exhausted")
            break
            
        if key in (ord('q'), ord('Q'), 27):
            break
            
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
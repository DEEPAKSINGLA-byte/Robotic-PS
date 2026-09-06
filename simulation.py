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

def calculate_travel_time(distance_pixels, payload):
    distance_meters = pixels_to_meters(distance_pixels)
    speed = calculate_speed(payload)
    if speed <= 0:
        return float("inf")
    return distance_meters / speed

def consume_battery(drone, distance):
    battery_used = calculate_energy_consumption(distance, drone.payload)
    drone.battery -= battery_used
    drone.battery = max(0.0, drone.battery)

def create_test_package():
    return PackageState(
        id=1, x=650.0, y=250.0, weight=1.0, deadline_remaining=60.0,
        assigned_drone=None, delivered=False
    )

def assign_test_drone(drone, package):
    drone.status = "DELIVERY"
    drone.target = (package.x, package.y)
    drone.payload = package.weight
    package.assigned_drone = drone.id

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
        consume_battery(drone, distance_moved)
        if drone.battery <= 0:
            drone.battery = 0
            drone.status = "FAILED"
            drone.target = None
            return
        if reached:
            package.delivered = True
            drone.status = "RETURNING"
            drone.target = (BASE[0], BASE[1])
    elif drone.status == "RETURNING":
        distance_moved, reached = move_towards(drone, drone.target, speed_pixels, dt)
        consume_battery(drone, distance_moved)
        if drone.battery <= 0:
            drone.battery = 0
            drone.status = "FAILED"
            drone.target = None
            return
        if reached:
            drone.status = "IDLE"
            drone.payload = 0.0
            drone.target = None

def run_simulation_step(drones, package, dt):
    for drone in drones.values():
        update_drone(drone, package, dt)

def initialize_drones():
    drones = {}
    for i in range(1, 11):
        drones[i] = DroneState(
            id=i, x=float(BASE[0]), y=float(BASE[1]), battery=100.0,
            status="IDLE", payload=0.0, target=None
        )
    return drones

def main():
    drones = initialize_drones()
    package = create_test_package()
    assign_test_drone(drones[1], package)
    env = Drone2DEnvironment(num_drones=10)
    cv2.namedWindow("Drone 2D Environment", cv2.WINDOW_NORMAL)
    dt = 0.1
    while True:
        run_simulation_step(drones, package, dt)
        env.update_state(
            drones=drones,
            packages={package.id: package},
            message="D1 status: {}".format(drones[1].status)
        )
        frame = env.draw()
        cv2.imshow("Drone 2D Environment", frame)
        key = cv2.waitKey(int(dt * 1000)) & 0xFF
        if package.delivered and drones[1].status == "IDLE":
            break
        if key in (ord('q'), ord('Q'), 27):
            break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
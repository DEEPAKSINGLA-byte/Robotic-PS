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
def consume_battery(drone, distance):
    """
    Reduce the drone's battery based on the distance travelled.
    """

    # Temporary model:
    # 100% battery = 2000 pixels of flight
    battery_used = (distance / 2000.0) * 100.0

    drone.battery -= battery_used

    # Prevent battery from becoming negative
    drone.battery = max(0.0, drone.battery)

def create_test_package():
    return PackageState(
        id=1,
        x=650.0,
        y=250.0,
        weight=1.0,
        deadline_remaining=60.0,
        assigned_drone=None,
        delivered=False
    )

def assign_test_drone(drone, package):
    drone.status = "DELIVERY"
    drone.target = (package.x, package.y)
    drone.payload = package.weight
    package.assigned_drone = drone.id

def update_drone(drone, package, dt):
    if drone.status == "DELIVERY":
        distance_moved, reached = move_towards(drone, drone.target, 150, dt)
        consume_battery(drone, distance_moved)
        if reached:
            package.delivered = True
            drone.status = "RETURNING"
            drone.target = (BASE[0], BASE[1])
    elif drone.status == "RETURNING":
        distance_moved, reached = move_towards(drone, drone.target, 150, dt)
        consume_battery(drone, distance_moved)
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
            id=i,
            x=float(BASE[0]),
            y=float(BASE[1]),
            battery=100.0,
            status="IDLE",
            payload=0.0,
            target=None
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
            message=f"D1 status: {drones[1].status}"
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
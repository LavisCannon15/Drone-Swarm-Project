import random
from dronekit import Vehicle, LocationGlobalRelative, VehicleMode
import time
import threading
from config import OFFSET_DISTANCE
from global_vars import stop_operations_event  # Import the event

def arm_and_takeoff(vehicle, target_altitude, drone_id):
    print(f"{drone_id}: Changing to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")

    while not vehicle.mode.name == "GUIDED":
        print(f"{drone_id}: Waiting for GUIDED mode...")
        if stop_operations_event.is_set():  # Check if the stop event is triggered
            print(f"{drone_id}: Stop operations signal received during mode change.")
            land(vehicle, drone_id)  # Land the drone if stopping
            return
        time.sleep(1)

    print(f"{drone_id}: Waiting for vehicle to be ready to arm...")
    while not vehicle.is_armable:
        print(f"{drone_id}: Vehicle not armable yet. Waiting...")
        if stop_operations_event.is_set():  # Check if the stop event is triggered
            print(f"{drone_id}: Stop operations signal received while checking arming status.")
            land(vehicle, drone_id)  # Land the drone if stopping
            return
        time.sleep(1)

    print(f"{drone_id}: Arming...")
    vehicle.armed = True

    while not vehicle.armed:
        print(f"{drone_id}: Waiting for arming...")
        if stop_operations_event.is_set():  # Check if the stop event is triggered
            print(f"{drone_id}: Stop operations signal received while waiting to arm.")
            land(vehicle, drone_id)  # Land the drone if stopping
            return
        time.sleep(1)

    print(f"{drone_id}: Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)

    # Monitor altitude during takeoff
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f"{drone_id}: Altitude: {altitude:.2f} meters")
        if stop_operations_event.is_set():  # Check if the stop event is triggered
            print(f"{drone_id}: Stop operations signal received during takeoff.")
            land(vehicle, drone_id)  # Land the drone if stopping
            return
        if altitude >= target_altitude * 0.95:
            print(f"{drone_id}: Reached target altitude.")
            break
        time.sleep(1)


def land(vehicle, drone_id):
    print(f"{drone_id}: Landing...")
    vehicle.mode = VehicleMode("LAND")

def calculate_triangle_positions(reference_lat, reference_lon, offset_distance):
    triangle_positions = [
        (reference_lat + offset_distance / 2, reference_lon),
        (reference_lat - offset_distance / 2, reference_lon + offset_distance / 2),
        (reference_lat - offset_distance / 2, reference_lon - offset_distance / 2)
    ]
    return triangle_positions

def move_to_positions(drones, triangle_positions):
    for drone, target_position in zip(drones, triangle_positions):
        drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'
        print(f"{drone_id}: Moving to triangle position at {target_position}...")
        drone.simple_goto(LocationGlobalRelative(target_position[0], target_position[1], drone.location.global_relative_frame.alt))

def simulate_user_movement(reference_lat, reference_lon):
    """Simulate user movement by adding a small random offset to their position."""
    delta_lat = random.uniform(-0.00001, 0.00001)
    delta_lon = random.uniform(-0.00001, 0.00001)
    return reference_lat + delta_lat, reference_lon + delta_lon

def operate_drones(drones, target_altitude, hover_time, reference_lat, reference_lon):
    # Arm and take off each drone
    threads = []
    for drone in drones:
        drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'
        thread = threading.Thread(target=arm_and_takeoff, args=(drone, target_altitude, drone_id))
        thread.start()
        threads.append(thread)

    # Wait for all arming and takeoff threads to finish
    for thread in threads:
        thread.join()

    # Allow a moment for all drones to stabilize after the takeoff command
    time.sleep(5)

    try:
        # Main loop: move drones based on simulated user movement
        while not stop_operations_event.is_set():
            # Calculate new positions based on current simulated user movement
            current_lat, current_lon = simulate_user_movement(reference_lat, reference_lon)
            triangle_positions = calculate_triangle_positions(current_lat, current_lon, OFFSET_DISTANCE)

            # Move drones to their new positions
            move_to_positions(drones, triangle_positions)

            # Hover for a short period (adjustable)
            time.sleep(hover_time)

    except Exception as e:
        print(f"Exception occurred: {e}")
    finally:
        # Ensure that the drones land regardless of the reason for stopping
        for drone in drones:
            drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'
            land(drone, drone_id)

        print("Drones have landed.")

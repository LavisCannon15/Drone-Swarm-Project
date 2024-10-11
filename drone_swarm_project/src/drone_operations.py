import random
from dronekit import Vehicle, LocationGlobalRelative, VehicleMode
import time
import threading
from config import OFFSET_DISTANCE
from global_vars import stop_operations_event
from geopy.distance import great_circle  # Ensure you have geopy installed

def arm_and_takeoff(vehicle, target_altitude, drone_id):
    print(f"{drone_id}: Changing to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    
    while not vehicle.mode.name == "GUIDED":
        print(f"{drone_id}: Waiting for GUIDED mode...")
        time.sleep(1)

    print(f"{drone_id}: Waiting for vehicle to be ready to arm...")
    while not vehicle.is_armable:
        print(f"{drone_id}: Vehicle not armable yet. Waiting...")
        time.sleep(1)

    print(f"{drone_id}: Arming...")
    vehicle.armed = True

    while not vehicle.armed:
        print(f"{drone_id}: Waiting for arming...")
        time.sleep(1)

    print(f"{drone_id}: Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f"{drone_id}: Altitude: {altitude:.2f} meters")
        if altitude >= target_altitude * 0.95:
            print(f"{drone_id}: Reached target altitude.")
            break
        time.sleep(1)

def land(vehicle, drone_id):
    print(f"{drone_id}: Landing...")
    vehicle.mode = VehicleMode("LAND")

def calculate_triangle_positions(reference_lat, reference_lon, offset_distance):
    # Calculate positions for equilateral triangle
    triangle_positions = [
        (reference_lat + offset_distance, reference_lon),  # Top drone
        (reference_lat - offset_distance / 2, reference_lon + (offset_distance * (3**0.5)) / 2),  # Bottom left drone
        (reference_lat - offset_distance / 2, reference_lon - (offset_distance * (3**0.5)) / 2)   # Bottom right drone
    ]
    return triangle_positions

def ensure_equal_distance(drones, triangle_positions, min_distance):
    # Ensure all drones maintain the minimum distance
    for i in range(len(drones)):
        for j in range(i + 1, len(drones)):
            drone1_pos = triangle_positions[i]
            drone2_pos = triangle_positions[j]
            distance = great_circle(drone1_pos, drone2_pos).meters


            if distance < min_distance:
                print(f"Adjusting positions for {drones[i].id} and {drones[j].id}")
                # Calculate the adjustment needed to maintain distance
                adjustment = (min_distance - distance) / 2
                # Adjust positions
                triangle_positions[i] = (
                    drone1_pos[0] + adjustment * (drone2_pos[0] - drone1_pos[0]) / distance,
                    drone1_pos[1] + adjustment * (drone2_pos[1] - drone1_pos[1]) / distance
                )
                triangle_positions[j] = (
                    drone2_pos[0] - adjustment * (drone2_pos[0] - drone1_pos[0]) / distance,
                    drone2_pos[1] - adjustment * (drone2_pos[1] - drone1_pos[1]) / distance
                )
    
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
    global stop_operations_event  # Use the global stop flag

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
        while not stop_operations_event.is_set():  # Check the event correctly
            # Calculate new positions based on current simulated user movement
            current_lat, current_lon = simulate_user_movement(reference_lat, reference_lon)
            triangle_positions = calculate_triangle_positions(current_lat, current_lon, OFFSET_DISTANCE)

            # Ensure drones are at equal distance
            triangle_positions = ensure_equal_distance(drones, triangle_positions, min_distance=5)  # 5 meters as an example

            # Move drones to their new positions
            move_to_positions(drones, triangle_positions)

            # Hover for a short period (adjustable)
            time.sleep(hover_time)

    except KeyboardInterrupt:
        # Handle keyboard interrupt
        print("KeyboardInterrupt detected, stopping drone operations.")
        stop_operations_event.set()  # Set the event correctly

    finally:
        # Ensure that the drones land regardless of the reason for stopping
        for drone in drones:
            drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'
            land(drone, drone_id)

        print("Drones have landed.")

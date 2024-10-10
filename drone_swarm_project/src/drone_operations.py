from dronekit import Vehicle, LocationGlobalRelative, VehicleMode
import time
import threading

def arm_and_takeoff(vehicle, target_altitude, drone_id):
    # Ensure the vehicle is in GUIDED mode before arming
    print(f"{drone_id}: Changing to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    
    # Wait for mode change
    while not vehicle.mode.name == "GUIDED":
        print(f"{drone_id}: Waiting for GUIDED mode...")
        time.sleep(1)

    # Check if the vehicle is ready to arm
    print(f"{drone_id}: Waiting for vehicle to be ready to arm...")
    while not vehicle.is_armable:
        print(f"{drone_id}: Vehicle not armable yet. Waiting...")
        time.sleep(1)

    # Arm the vehicle
    print(f"{drone_id}: Arming...")
    vehicle.armed = True

    # Wait for arming confirmation
    while not vehicle.armed:
        print(f"{drone_id}: Waiting for arming...")
        time.sleep(1)

    # Takeoff command
    print(f"{drone_id}: Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)

    # Wait until the vehicle reaches a safe height before continuing
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
    # Define the triangle formation around the reference point
    triangle_positions = [
        (reference_lat + offset_distance / 2, reference_lon),  # Vertex 1
        (reference_lat - offset_distance / 2, reference_lon + offset_distance / 2),  # Vertex 2
        (reference_lat - offset_distance / 2, reference_lon - offset_distance / 2)  # Vertex 3
    ]
    return triangle_positions

def move_to_positions(drones, triangle_positions):
    for drone, target_position in zip(drones, triangle_positions):
        drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'
        print(f"{drone_id}: Moving to triangle position at {target_position}...")
        drone.simple_goto(LocationGlobalRelative(target_position[0], target_position[1], drone.location.global_relative_frame.alt))

def operate_drones(drones, target_altitude, hover_time, reference_lat, reference_lon):
    # Create a list to hold threads
    threads = []

    # Arm and take off each drone in its own thread
    for drone in drones:
        drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'
        thread = threading.Thread(target=arm_and_takeoff, args=(drone, target_altitude, drone_id))
        thread.start()  # Start the thread
        threads.append(thread)  # Keep track of the threads

    # Wait for all arming and takeoff threads to finish
    for thread in threads:
        thread.join()  # Wait for the thread to complete

    # Allow a moment for all drones to stabilize after the takeoff command
    time.sleep(5)

    # Calculate the offset distance for 3 meters apart
    desired_distance_m = 3  # Desired distance in meters
    offset_distance = desired_distance_m / 111139  # Convert meters to degrees

    # Calculate triangle positions based on the reference coordinates
    triangle_positions = calculate_triangle_positions(reference_lat, reference_lon, offset_distance)

    # Move drones to their respective triangle positions
    move_to_positions(drones, triangle_positions)

    # Hover for a specific time
    time.sleep(hover_time)

    # Land all drones
    for drone in drones:
        drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'
        land(drone, drone_id)

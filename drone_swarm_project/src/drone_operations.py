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

    print(f"{drone_id}: Arming...")
    vehicle.armed = True

    # Wait for arming confirmation
    while not vehicle.armed:
        print(f"{drone_id}: Waiting for arming...")
        time.sleep(1)

    print(f"{drone_id}: Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f"{drone_id}: Altitude: {altitude:.2f} meters", end='\r')
        if altitude >= target_altitude * 0.95:
            print(f"\n{drone_id}: Reached target altitude")
            break
        time.sleep(1)

def land(vehicle, drone_id):
    print(f"{drone_id}: Landing...")
    vehicle.mode = VehicleMode("LAND")

def calculate_triangle_positions(reference_lat, reference_lon, offset_distance):
    # Define the triangle formation around the reference point
    triangle_positions = [
        (reference_lat + offset_distance, reference_lon),  # Vertex 1
        (reference_lat - offset_distance, reference_lon + offset_distance),  # Vertex 2
        (reference_lat - offset_distance, reference_lon - offset_distance)  # Vertex 3
    ]
    
    return triangle_positions

def move_to_positions(drones, triangle_positions):
    for drone, target_position in zip(drones, triangle_positions):
        print(f"{drone}: Moving to triangle position at {target_position}...")
        drone.simple_goto(LocationGlobalRelative(target_position[0], target_position[1], drone.location.global_relative_frame.alt))

def operate_drones(drones, target_altitude, hover_time, reference_lat, reference_lon):
    threads = []

    # Arm and take off each drone simultaneously
    for drone in drones:
        thread = threading.Thread(target=arm_and_takeoff, args=(drone, target_altitude, drone))
        threads.append(thread)
        thread.start()

    # Wait for all drones to take off
    for thread in threads:
        thread.join()

    # Allow some time for all drones to take off
    time.sleep(5)

    # Calculate triangle positions based on the reference coordinates
    offset_distance = 0.00005  # Adjust as necessary (in degrees) for closer formation
    triangle_positions = calculate_triangle_positions(reference_lat, reference_lon, offset_distance)

    # Move drones to their respective triangle positions
    move_to_positions(drones, triangle_positions)

    # Hover for a specific time
    time.sleep(hover_time)

    # Land all drones
    for drone in drones:
        land(drone, drone)

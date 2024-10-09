from dronekit import connect, LocationGlobalRelative
from drone_operations import arm_and_takeoff, land
from config import DRONE_CONNECTIONS, TRIANGLE_POINTS
import threading

# Function to handle the operations of each drone
def operate_drone(vehicle, target_altitude, hover_time, drone_id, target_location):
    arm_and_takeoff(vehicle, target_altitude, hover_time, drone_id)
    print(f"{drone_id}: Moving to triangle formation location...")
    
    # Move to the specified target location
    vehicle.simple_goto(LocationGlobalRelative(target_location[0], target_location[1], target_location[2]))
    
    # Wait until the drone reaches the target location
    while True:
        current_location = vehicle.location.global_relative_frame
        distance = ((current_location.lat - target_location[0])**2 + (current_location.lon - target_location[1])**2)**0.5
        print(f"{drone_id}: Distance to target: {distance:.2f} meters", end='\r')
        if distance < 1:  # Adjust the threshold as needed
            print(f"\n{drone_id}: Reached formation point")
            break
        time.sleep(1)
    
    land(vehicle, drone_id)  # Land after reaching the target

# Connect to each drone using configuration
vehicles = {}
for drone_id, connection in DRONE_CONNECTIONS.items():
    print(f"Connecting to {drone_id} on: {connection}")
    vehicles[drone_id] = connect(connection, wait_ready=True)
    print(f"{drone_id} connected")

# Create threads for each drone's operations with specific target locations
threads = []
for idx, (drone_id, vehicle) in enumerate(vehicles.items()):
    thread = threading.Thread(target=operate_drone, args=(vehicle, 10, 10 + idx * 5, drone_id, TRIANGLE_POINTS[idx]))
    threads.append(thread)

# Start all threads
for thread in threads:
    thread.start()

# Wait for all threads to complete
for thread in threads:
    thread.join()

# Close connections
for vehicle in vehicles.values():
    vehicle.close()

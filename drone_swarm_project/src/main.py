from dronekit import connect
from drone_operations import operate_drones
from config import DRONE_CONNECTIONS, TAKEOFF_ALTITUDE, HOVER_TIME, USER_LATITUDE, USER_LONGITUDE
from global_vars import stop_operations_event  # Import the event
import threading
import signal
import sys

def signal_handler(sig, frame):
    print("Signal received! Stopping drone operations...")
    stop_operations_event.set()  # Set the event to True

# Register signal handler for graceful shutdown
signal.signal(signal.SIGINT, signal_handler)

# Connect to each drone using configuration
vehicles = {}
for drone_id, connection in DRONE_CONNECTIONS.items():
    print(f"Connecting to {drone_id} on: {connection}")
    vehicle = connect(connection, wait_ready=True)
    vehicle.id = drone_id  # Assign the drone_id to the vehicle
    vehicles[drone_id] = vehicle
    print(f"{drone_id} connected")

# Create a list of vehicles for operate_drones function
drone_list = list(vehicles.values())

# Start the drone operations in a thread
drone_operation_thread = threading.Thread(target=operate_drones, args=(drone_list, TAKEOFF_ALTITUDE, USER_LATITUDE, USER_LONGITUDE))
drone_operation_thread.start()

# Main loop: wait for the drone operations to complete or stop
try:
    while not stop_operations_event.is_set():
        drone_operation_thread.join(timeout=1)
except KeyboardInterrupt:
    print("Keyboard interrupt received! Stopping drone operations...")
    stop_operations_event.set()  # Gracefully stop the drones

# Ensure drones have stopped and landed
drone_operation_thread.join()  # Wait for the drone operations thread to finish

# Close all vehicle connections
for vehicle in vehicles.values():
    vehicle.close()

print("Drone connections closed. Exiting.")
#main.py

from dronekit import connect
from drone_operations import operate_drones
from config import (
    DRONE_CONNECTIONS,
    TAKEOFF_ALTITUDE,
    USER_LATITUDE,
    USER_LONGITUDE,
    OFFSET_DISTANCE,
    ORBIT_AROUND_USER,
    SWAP_POSITIONS,
    SWAP_POSITION_SPEED,
    ROTATE_TRIANGLE_FORMATION,
    ANGLE_OFFSET,
    REVOLVE_SPEED,
    REVOLVE_OFFSET_DISTANCE,
)
from global_vars import stop_operations_event  # Import the event
import signal

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

# Run the drone operations directly
try:
    operate_drones(
        drone_list,
        TAKEOFF_ALTITUDE,
        USER_LATITUDE,
        USER_LONGITUDE,
        OFFSET_DISTANCE,
        ORBIT_AROUND_USER,
        SWAP_POSITIONS,
        SWAP_POSITION_SPEED,
        ROTATE_TRIANGLE_FORMATION,
        ANGLE_OFFSET,
        REVOLVE_SPEED,
        REVOLVE_OFFSET_DISTANCE,
    )
except KeyboardInterrupt:
    print("Keyboard interrupt received! Stopping drone operations...")
    stop_operations_event.set()  # Gracefully stop the drones

# Ensure drones have stopped and landed (this logic should be in operate_drones)
for vehicle in vehicles.values():
    vehicle.close()

print("Drone connections closed. Exiting.")

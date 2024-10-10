from dronekit import connect
from drone_operations import operate_drones
from config import DRONE_CONNECTIONS
import threading

# Define takeoff altitude and hover time
TAKEOFF_ALTITUDE = 2  # meters
HOVER_TIME = 10       # seconds

# User's location (update these to the user's actual coordinates)
USER_LATITUDE = -35.3631723  # Example latitude
USER_LONGITUDE = 149.1652367  # Example longitude

# Connect to each drone using configuration
vehicles = {}
for drone_id, connection in DRONE_CONNECTIONS.items():
    print(f"Connecting to {drone_id} on: {connection}")
    vehicles[drone_id] = connect(connection, wait_ready=True)
    print(f"{drone_id} connected")

# Create a list of vehicles for operate_drones function
drone_list = list(vehicles.values())

# Start the drone operations in a thread
drone_operation_thread = threading.Thread(target=operate_drones, args=(drone_list, TAKEOFF_ALTITUDE, HOVER_TIME, USER_LATITUDE, USER_LONGITUDE))
drone_operation_thread.start()

# Wait for the drone operations to complete
drone_operation_thread.join()

# Close connections
for vehicle in vehicles.values():
    vehicle.close()

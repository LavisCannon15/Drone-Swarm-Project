# config.py
# Configuration file for drone settings

# Define takeoff altitude and hover time
TAKEOFF_ALTITUDE = 2  # meters
HOVER_TIME = 3       # seconds

# User's location (update these to the user's actual coordinates)
USER_LATITUDE = -35.3631723  # Example latitude
USER_LONGITUDE = 149.1652367  # Example longitude




# Offset distance for drone positions 
OFFSET_DISTANCE = 4  # Convert meters to degrees

ORBIT_AROUND_USER = True # Activates/Deactivates revolvement

REVOLVE_OFFSET_DISTANCE = 4
ANGLE_OFFSET = 0 #Starting Point (like the position of the hour hand)
REVOLVE_SPEED = 40 #Speed of revolution in m/s (Adjust this value for faster or slower revolving)


ROTATE_TRIANGLE_FORMATION = False


SWAP_POSITIONS = False
SWAP_POSITION_SPEED = 3






# Drone connection strings (example configuration)
DRONE_CONNECTIONS = {
    "drone_1": "127.0.0.1:14550",
    "drone_2": "127.0.0.1:14560",
    "drone_3": "127.0.0.1:14570"
}
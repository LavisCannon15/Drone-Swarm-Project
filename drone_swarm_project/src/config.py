# Configuration file for drone settings

# Define takeoff altitude and hover time
TAKEOFF_ALTITUDE = 2  # meters
HOVER_TIME = 3       # seconds

# User's location (update these to the user's actual coordinates)
USER_LATITUDE = -35.3631723  # Example latitude
USER_LONGITUDE = 149.1652367  # Example longitude

# Offset distance for drone positions (in meters, converted to degrees)
DESIRED_DISTANCE_M = 3  # meters
OFFSET_DISTANCE = DESIRED_DISTANCE_M / 111139  # Convert meters to degrees

# Drone connection strings (example configuration)
DRONE_CONNECTIONS = {
    "drone_1": "127.0.0.1:14550",
    "drone_2": "127.0.0.1:14560",
    "drone_3": "127.0.0.1:14570"
}

# config.py

# Define connection strings for each drone (using UDP)
DRONE_CONNECTIONS = {
    "drone1": 'udp:127.0.0.1:14550',
    "drone2": 'udp:127.0.0.1:14560',
    "drone3": 'udp:127.0.0.1:14570',
}

# Triangle formation coordinates (latitude, longitude, altitude)
TRIANGLE_POINTS = [
    (37.7925, -122.3959, 1),  # Drone 1
    (37.7927, -122.3965, 1),  # Drone 2
    (37.7923, -122.3961, 1)   # Drone 3
]

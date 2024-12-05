import math
import matplotlib.pyplot as plt
import numpy as np
import random
import time

def read_accelerometer_data():
    """Simulate reading data from the MPU6050 accelerometer with noise."""
    noise = random.uniform(-0.05, 0.05)  # Simulated noise
    ax = random.uniform(-1.0, 1.0) + noise  # ±1g range
    ay = random.uniform(0.5, 1.5) + noise    # Simulate forward movement
    az = random.uniform(-0.1, 0.1) + noise    # Simulate downward movement
    return ax, ay, az

def read_gyroscope_data():
    """Simulate reading data from the MPU6050 gyroscope with drift."""
    gx_drift = 0.01  # Small drift per second
    gy_drift = 0.01  # Small drift per second
    gz_drift = 0.01  # Small drift per second

    # Simulate gyroscope readings for rotations around X, Y, Z axes
    gx = random.uniform(-5.0, 5.0) + gx_drift
    gy = random.uniform(-5.0, 5.0) + gy_drift
    gz = random.uniform(-5.0, 5.0) + gz_drift
    return gx, gy, gz

def read_gps_data(reference_lat, reference_lon):
    """Simulate a GPS reading based on a reference latitude and longitude with noise."""
    gps_lat_noise = random.uniform(-0.00001, 0.00001)
    gps_lon_noise = random.uniform(-0.00001, 0.00001)
    gps_lat = reference_lat + gps_lat_noise
    gps_lon = reference_lon + gps_lon_noise
    return gps_lat, gps_lon

def read_compass_data():
    """Simulate reading data from a compass (magnetometer) with noise."""
    noise = random.uniform(-2, 2)  # Simulated noise
    heading = (random.uniform(0, 360) + noise) % 360
    return heading
def kalman_filter(state, P, measurement, accel, gyro, compass, dt):
    """Perform one iteration of the Kalman filter with extended data."""
    # State transition matrix (A)
    A = np.array([
        [1, 0, dt * np.cos(np.radians(compass)), 0],  # Update latitude based on velocity_lat
        [0, 1, dt * np.sin(np.radians(compass)), 0],  # Update longitude based on velocity_lon
        [0, 0, 1, 0],   # Velocity_lat remains the same in the absence of acceleration
        [0, 0, 0, 1]    # Velocity_lon remains the same in the absence of acceleration
    ])
    
    # Control input model (B)
    B = np.array([
        [0.5 * dt ** 2, 0],
        [0, 0.5 * dt ** 2],
        [dt, 0],
        [0, dt]
    ])
    
    # Control vector (acceleration in lat and lon directions)
    u = np.array([accel[1], accel[0]])  # ax affects lon, ay affects lat
    
    # Process noise covariance (Q)
    Q = np.eye(4) * 0.001  # Small uncertainty in prediction
    
    # Predict next state
    state = A @ state + B @ u
    P = A @ P @ A.T + Q
    
    # Measurement update (Z) - actual GPS measurements
    Z = np.array([measurement[0], measurement[1]])  # [lat, lon]
    
    # Measurement matrix (H)
    H = np.array([
        [1, 0, 0, 0],  # Measure latitude directly
        [0, 1, 0, 0]   # Measure longitude directly
    ])
    
    # Measurement noise covariance (R)
    R = np.eye(2) * 0.01  # GPS measurement noise
    
    # Innovation or measurement residual (Y)
    Y = Z - H @ state
    
    # Innovation covariance (S)
    S = H @ P @ H.T + R
    
    # Kalman gain (K)
    K = P @ H.T @ np.linalg.inv(S)
    
    # Update state estimate and covariance
    state = state + K @ Y
    P = (np.eye(4) - K @ H) @ P

    # Update state with gyroscope data for orientation changes (if necessary)
    state[2] += gyro[0] * dt  # Update velocity_lat based on gx
    state[3] += gyro[1] * dt  # Update velocity_lon based on gy

    return state, P

def simulate_user_movement(reference_lat, reference_lon, pause_duration=0.5):
    """Simulate user movement using GPS, accelerometer, gyroscope, and compass data."""
    state = np.array([reference_lat, reference_lon, 0, 0])
    P = np.eye(4)
    dt = pause_duration
    gps_lat, gps_lon = read_gps_data(reference_lat, reference_lon)
    ax, ay, az = read_accelerometer_data()
    gx, gy, gz = read_gyroscope_data()
    heading = read_compass_data()

    state, P = kalman_filter(state, P, (gps_lat, gps_lon), (ax, ay), (gx, gy, gz), heading, dt)

    new_lat = state[0]
    new_lon = state[1]
    kalman_speed_x = state[2]
    kalman_speed_y = state[3]
    kalman_user_speed = np.sqrt(kalman_speed_x**2 + kalman_speed_y**2)

    time.sleep(pause_duration)
    return new_lat, new_lon, kalman_user_speed

def calculate_triangle_positions(reference_lat, reference_lon, offset_distance):
    # Calculate positions for equilateral triangle
    triangle_positions = [
        (reference_lat + offset_distance, reference_lon),  # Top drone
        (reference_lat - offset_distance / 2, reference_lon + (offset_distance * (3**0.5)) / 2),  # Bottom left drone
        (reference_lat - offset_distance / 2, reference_lon - (offset_distance * (3**0.5)) / 2)   # Bottom right drone
    ]

    #print("Triangle positions:", str(triangle_positions))
    return triangle_positions


def rotate_triangle_positions(positions):
    """
    Rotate the positions of the drones.
    
    :param positions: List of tuples representing the current positions of the drones.
                      [(lat1, lon1), (lat2, lon2), (lat3, lon3)]
    :return: List of tuples representing the new positions after rotation.
    """
    if len(positions) != 3:
        raise ValueError("The positions list must contain exactly three tuples.")
    
    # Rotate positions
    rotated_positions = [positions[2], positions[0], positions[1]]
    return rotated_positions




def calculate_revolving_positions(current_lat, current_lon, offset_distance, num_drones, angle_offset):
    # List to store the calculated positions
    positions = []
    
    # Loop through the number of drones
    for i in range(num_drones):
        # Calculate the angle for each drone in radians
        angle_rad = math.radians(angle_offset+ i * (360 / num_drones))  # Evenly distribute angles
        
        # Calculate new latitude and longitude
        new_lat = current_lat + (offset_distance / 111320) * math.cos(angle_rad)  # For latitude
        new_lon = current_lon + (offset_distance / (111320 * math.cos(math.radians(current_lat)))) * math.sin(angle_rad)  # For longitude

        # Append the new position to the list
        positions.append((new_lat, new_lon))

    #print("Reolving positions:", str(positions))

    return positions


def rotate_triangle_around_center(positions, angle_degrees):
    """
    Rotates the triangular formation of drones around its center.
    
    :param positions: List of tuples representing the current positions of the drones.
                      [(lat1, lon1), (lat2, lon2), (lat3, lon3)]
    :param angle_degrees: The angle in degrees to rotate the triangle.
    :return: List of tuples representing the new positions after rotation.
    """
    # Convert the angle from degrees to radians
    angle_radians = math.radians(angle_degrees)
    
    # Calculate the centroid of the triangle (average of the points)
    center_lat = sum(lat for lat, lon in positions) / len(positions)
    center_lon = sum(lon for lat, lon in positions) / len(positions)
    
    # Rotate each drone's position around the centroid
    rotated_positions = []
    for lat, lon in positions:
        # Calculate the position relative to the center
        relative_lat = lat - center_lat
        relative_lon = lon - center_lon
        
        # Apply rotation formula
        rotated_lat = relative_lat * math.cos(angle_radians) - relative_lon * math.sin(angle_radians)
        rotated_lon = relative_lat * math.sin(angle_radians) + relative_lon * math.cos(angle_radians)
        
        # Translate back to the original center
        new_lat = rotated_lat + center_lat
        new_lon = rotated_lon + center_lon
        rotated_positions.append((new_lat, new_lon))
    
    return rotated_positions



# Simulation parameters
reference_lat = -35.3631723
reference_lon = 149.1652367
offset_distance = 5
num_drones = 3
iterations = 100
angle_increment = 3

all_positions = []
angle_offset = 0

for i in range(iterations):
    current_lat, current_lon, kalman_user_speed = simulate_user_movement(reference_lat, reference_lon)

    positions = calculate_revolving_positions(current_lat, current_lon, offset_distance, num_drones, angle_offset)
    angle_offset += angle_increment

    #positions = calculate_triangle_positions(current_lat, current_lon, offset_distance)
    #positions = rotate_triangle_positions(positions)



    for lat, lon in positions:
        x = (lon - current_lon) * 111320 * math.cos(math.radians(current_lat))
        y = (lat - current_lat) * 111320
        all_positions.append((x, y))
    
    

x_values = [pos[0] for pos in all_positions]
y_values = [pos[1] for pos in all_positions]

plt.figure(figsize=(10, 10))
plt.scatter(x_values, y_values, c='blue', marker='o', label='Drone Positions')
plt.title('Drone Swarm Orbiting with Simulated User Movement')
plt.xlabel('X Coordinates (meters)')
plt.ylabel('Y Coordinates (meters)')
plt.axhline(0, color='gray', linewidth=0.5, linestyle='--')
plt.axvline(0, color='gray', linewidth=0.5, linestyle='--')
plt.grid()
plt.legend()
plt.axis('equal')
plt.show()

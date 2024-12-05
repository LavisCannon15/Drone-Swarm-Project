import math
import numpy as np
import random
import pandas as pd
import time

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

    return positions

def read_accelerometer_data():
    """Simulate reading data from the MPU6050 accelerometer with noise."""
    noise = random.uniform(-0.05, 0.05)
    ax = random.uniform(-1.0, 1.0) + noise
    ay = random.uniform(0.5, 1.5) + noise
    az = random.uniform(-0.1, 0.1) + noise
    return ax, ay, az

def read_gyroscope_data():
    """Simulate reading data from the MPU6050 gyroscope with drift."""
    gx_drift = 0.01
    gy_drift = 0.01
    gz_drift = 0.01

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
    noise = random.uniform(-2, 2)
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
    P = np.eye(4)  # Initial uncertainty
    dt = pause_duration

    gps_lat, gps_lon = read_gps_data(reference_lat, reference_lon)
    ax, ay, az = read_accelerometer_data()
    gx, gy, gz = read_gyroscope_data()
    heading = read_compass_data()

    state, P = kalman_filter(state, P, (gps_lat, gps_lon), (ax, ay), (gx, gy, gz), heading, dt)

    new_lat = state[0]
    new_lon = state[1]

    return new_lat, new_lon

# Main simulation parameters
current_lat = -35.3631723
current_lon = 149.1652367
offset_distance = 5  # Offset distance in meters
num_drones = 3
angle_offset = 0
angle_increment = 5  # Adjusted for a circular pattern

# Data storage for drone positions
all_drones_positions = []

# Simulate for a number of iterations
for _ in range(100):
    # Simulate user movement
    new_lat, new_lon = simulate_user_movement(current_lat, current_lon)
    
    # Calculate the positions of the drones
    drone_positions = calculate_revolving_positions(new_lat, new_lon, offset_distance, num_drones, angle_offset)
    angle_offset += angle_increment
    
    # Append the current drone positions to the data storage
    all_drones_positions.append(drone_positions)

# Create a DataFrame to display the results
df = pd.DataFrame(all_drones_positions, columns=[f'Drone {i+1} (Lat, Lon)' for i in range(num_drones)])
# Display the DataFrame
print(df)
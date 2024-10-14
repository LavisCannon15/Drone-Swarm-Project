import random
from dronekit import Vehicle, LocationGlobalRelative, VehicleMode
import time
import threading
from config import OFFSET_DISTANCE
from global_vars import stop_operations_event
from geopy.distance import great_circle  # Ensure you have geopy installed
import numpy as np

def arm_and_takeoff(vehicle, target_altitude, drone_id):
    print(f"{drone_id}: Changing to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    
    while not vehicle.mode.name == "GUIDED":
        print(f"{drone_id}: Waiting for GUIDED mode...")
        time.sleep(1)

    print(f"{drone_id}: Waiting for vehicle to be ready to arm...")
    while not vehicle.is_armable:
        print(f"{drone_id}: Vehicle not armable yet. Waiting...")
        time.sleep(1)

    print(f"{drone_id}: Arming...")
    vehicle.armed = True

    while not vehicle.armed:
        print(f"{drone_id}: Waiting for arming...")
        time.sleep(1)

    print(f"{drone_id}: Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f"{drone_id}: Altitude: {altitude:.2f} meters")
        if altitude >= target_altitude * 0.95:
            print(f"{drone_id}: Reached target altitude.")
            break
        time.sleep(1)

def land(vehicle, drone_id):
    print(f"{drone_id}: Landing...")
    vehicle.mode = VehicleMode("LAND")

def calculate_triangle_positions(reference_lat, reference_lon, offset_distance):
    # Calculate positions for equilateral triangle
    triangle_positions = [
        (reference_lat + offset_distance, reference_lon),  # Top drone
        (reference_lat - offset_distance / 2, reference_lon + (offset_distance * (3**0.5)) / 2),  # Bottom left drone
        (reference_lat - offset_distance / 2, reference_lon - (offset_distance * (3**0.5)) / 2)   # Bottom right drone
    ]
    return triangle_positions

def ensure_equal_distance(drones, triangle_positions, min_distance):
    # Ensure all drones maintain the minimum distance
    for i in range(len(drones)):
        for j in range(i + 1, len(drones)):
            drone1_pos = triangle_positions[i]
            drone2_pos = triangle_positions[j]
            distance = great_circle(drone1_pos, drone2_pos).meters

            if distance < min_distance:
                print(f"Adjusting positions for {drones[i].id} and {drones[j].id}")
                # Calculate the adjustment needed to maintain distance
                adjustment = (min_distance - distance) / 2
                # Adjust positions
                triangle_positions[i] = (
                    drone1_pos[0] + adjustment * (drone2_pos[0] - drone1_pos[0]) / distance,
                    drone1_pos[1] + adjustment * (drone2_pos[1] - drone1_pos[1]) / distance
                )
                triangle_positions[j] = (
                    drone2_pos[0] - adjustment * (drone2_pos[0] - drone1_pos[0]) / distance,
                    drone2_pos[1] - adjustment * (drone2_pos[1] - drone1_pos[1]) / distance
                )
    
    return triangle_positions


def move_to_positions(drones, triangle_positions,kalman_user_speed):
    for drone, target_position in zip(drones, triangle_positions):
        drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'
        #print(f"{drone_id}: Moving to triangle position at {target_position}...")
        drone.airspeed = kalman_user_speed
        drone.simple_goto(LocationGlobalRelative(target_position[0], target_position[1], drone.location.global_relative_frame.alt))
        print(f"{drone_id}: Speed {drone.airspeed:.2f} m/s (User Kalman Speed: {kalman_user_speed:.2f} m/s)")
    


def read_accelerometer_data():
    """Simulate reading data from the MPU6050 accelerometer."""
    # Simulate accelerometer readings for X, Y, Z axes
    ax = random.uniform(-0.5, 0.5)  # Simulate side-to-side movements
    ay = random.uniform(0.5, 1.0)    # Simulate forward movement
    az = random.uniform(-0.1, 0.1)    # Simulate downward movement
    return ax, ay, az

def read_gyroscope_data():
    """Simulate reading data from the MPU6050 gyroscope."""
    # Simulate gyroscope readings for rotations around X, Y, Z axes
    gx = random.uniform(-5.0, 5.0)  # Simulate roll rate (degrees/sec)
    gy = random.uniform(-5.0, 5.0)  # Simulate pitch rate (degrees/sec)
    gz = random.uniform(-5.0, 5.0)  # Simulate yaw rate (degrees/sec)
    return gx, gy, gz

def read_gps_data(reference_lat, reference_lon):
    """Simulate a GPS reading based on a reference latitude and longitude."""
    gps_lat = reference_lat + random.uniform(-0.00001, 0.00001)
    gps_lon = reference_lon + random.uniform(-0.00001, 0.00001)
    return gps_lat, gps_lon
   

def read_compass_data():
    """Simulate reading data from a compass (magnetometer)."""
    # Simulate a heading value representing the orientation in degrees (0-360)
    heading = random.uniform(0, 360)
    return heading

def kalman_filter(state, P, measurement, accel, gyro, compass, dt):
    """Perform one iteration of the Kalman filter with extended data.

    state: The current state (lat, lon, v_lat, v_lon).
    P: The current state covariance matrix.
    measurement: The GPS measurement (lat, lon).
    accel: The accelerometer data (ax, ay).
    gyro: The gyroscope data (gx, gy, gz).
    compass: The compass heading in degrees.
    dt: Time step between updates.
    """
    # State transition matrix (A)
    A = np.array([
        [1, 0, dt, 0],  # Update latitude based on velocity_lat * dt
        [0, 1, 0, dt],  # Update longitude based on velocity_lon * dt
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

    # Integrate gyroscope and compass data if needed (e.g., for orientation adjustments)
    # Here, you can use gx, gy, gz, and heading to refine position or orientation tracking.
    
    return state, P

def simulate_user_movement(reference_lat, reference_lon, pause_duration=0.0):
    """Simulate user movement using GPS, accelerometer, gyroscope, and compass data."""
    # Initial state: [latitude, longitude, velocity_lat, velocity_lon]
    state = np.array([reference_lat, reference_lon, 0, 0])
    P = np.eye(4)  # Initial uncertainty

    # Time step (in seconds)
    dt = pause_duration

    # Read accelerometer, gyroscope, GPS and compass data
    gps_lat, gps_lon = read_gps_data(reference_lat, reference_lon)
    ax, ay, az = read_accelerometer_data()
    gx, gy, gz = read_gyroscope_data()
    heading = read_compass_data()

    # Update the state using the Kalman filter
    state, P = kalman_filter(state, P, (gps_lat, gps_lon), (ax, ay), (gx, gy, gz), heading, dt)

    # Extract updated latitude and longitude from the state
    new_lat = state[0]
    new_lon = state[1]

    # Extract speed from Kalman filter state (state[2] and state[3] are velocities)
    kalman_speed_x = state[2]
    kalman_speed_y = state[3]

    # Output Kalman speed (m/s)
    kalman_user_speed = np.sqrt(kalman_speed_x**2 + kalman_speed_y**2)
    #print(f"Kalman Speed (User): {kalman_user_speed:.2f} m/s")

    # Pause for a specified duration between movements
    time.sleep(pause_duration)

    return new_lat, new_lon, kalman_user_speed  # Return Kalman speed

def operate_drones(drones, target_altitude, reference_lat, reference_lon):
    global stop_operations_event  # Use the global stop flag

    # Arm and take off each drone
    threads = []
    for drone in drones:
        drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'
        thread = threading.Thread(target=arm_and_takeoff, args=(drone, target_altitude, drone_id))
        thread.start()
        threads.append(thread)

    # Wait for all arming and takeoff threads to finish
    for thread in threads:
        thread.join()

    # Allow a moment for all drones to stabilize after the takeoff command
    time.sleep(5)

    try:
        # Main loop: move drones based on simulated user movement
        while not stop_operations_event.is_set():
            # Simulate user movement (you would replace this with real GPS data for the user)
            current_lat, current_lon, kalman_user_speed = simulate_user_movement(reference_lat, reference_lon, pause_duration=1.0)
            
            # Calculate the triangle positions around the user's updated location
            triangle_positions = calculate_triangle_positions(current_lat, current_lon, OFFSET_DISTANCE)

            # Move drones to their new positions
            move_to_positions(drones, triangle_positions, kalman_user_speed)

            # Short sleep to give time for drones to adjust
            time.sleep(1)

    except KeyboardInterrupt:
        # Handle keyboard interrupt
        print("KeyboardInterrupt detected, stopping drone operations.")
        stop_operations_event.set()

    except TimeoutError:
        # Handle communication timeouts or delays
        print("TimeoutError: Communication with the drone timed out. Initiating landing sequence.")
        stop_operations_event.set()

    except ValueError as e:
        # Handle data-related issues (e.g., invalid coordinates or speed)
        print(f"ValueError: {e}. Initiating landing sequence for safety.")
        stop_operations_event.set()

    except Exception as e:
        # Handle any other unexpected errors
        print(f"Unexpected error: {e}. Initiating landing sequence.")
        stop_operations_event.set()

    finally:
        # Ensure that the drones land regardless of the reason for stopping
        for drone in drones:
            drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'
            try:
                land(drone, drone_id)
            except Exception as e:
                print(f"Error during landing of {drone_id}: {e}")
                
        print("Drones have landed safely.")

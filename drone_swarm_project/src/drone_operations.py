#drone_operations.py

import random
import dronekit
from dronekit import LocationGlobalRelative, VehicleMode
from pymavlink import mavutil
import time
import threading
from global_vars import stop_operations_event
from geopy.distance import great_circle  # Ensure you have geopy installed
import numpy as np
from error_handler import monitor_drones, handle_drone_exceptions
import math


def arm_and_takeoff(vehicle, target_altitude, drone_id, stop_operations_event):
    print(f"{drone_id}: Changing to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    
    # Wait until the vehicle is in GUIDED mode
    while not vehicle.mode.name == "GUIDED":
        if stop_operations_event.is_set():
            print(f"{drone_id}: Stop signal received. Aborting mode change.")
            return
        print(f"{drone_id}: Waiting for GUIDED mode...")
        time.sleep(1)

    print(f"{drone_id}: Waiting for vehicle to be ready to arm...")
    while not vehicle.is_armable:
        if stop_operations_event.is_set():
            print(f"{drone_id}: Stop signal received. Aborting arming process.")
            return
        print(f"{drone_id}: Vehicle not armable yet. Waiting...")
        time.sleep(1)

    print(f"{drone_id}: Arming...")
    vehicle.armed = True

    while not vehicle.armed:
        if stop_operations_event.is_set():
            print(f"{drone_id}: Stop signal received. Aborting arming.")
            return
        print(f"{drone_id}: Waiting for arming...")
        time.sleep(1)

    print(f"{drone_id}: Taking off to {target_altitude} meters...")
    vehicle.simple_takeoff(target_altitude)

    while True:
        if stop_operations_event.is_set():
            print(f"{drone_id}: Stop signal received. Aborting takeoff.")
            return
        
        altitude = vehicle.location.global_relative_frame.alt
        print(f"{drone_id}: Altitude: {altitude:.2f} meters")
        
        # Exit the loop when the target altitude is reached
        if altitude >= target_altitude * 0.95:
            print(f"{drone_id}: Reached target altitude.")
            break
        
        time.sleep(1)


def land(vehicle, drone_id):
    print(f"{drone_id}: Landing...")
    vehicle.mode = VehicleMode("LAND")


def calculate_triangle_positions(reference_lat, reference_lon, offset_distance):
    # Convert offset distance from degrees to meters
    offset_distance_meters = offset_distance / 111320  # Conversion factor for degrees to meters

    # Calculate positions for equilateral triangle using meters
    triangle_positions = [
        (reference_lat + offset_distance_meters, reference_lon),  # Top drone
        (reference_lat - offset_distance_meters / 2, reference_lon + (offset_distance_meters * (3**0.5)) / 2),  # Bottom left drone
        (reference_lat - offset_distance_meters / 2, reference_lon - (offset_distance_meters * (3**0.5)) / 2)   # Bottom right drone
    ]

    return triangle_positions


def swap_triangle_positions(positions, counter):
    # Assuming positions is a list of tuples: [(x1, y1), (x2, y2), (x3, y3)]
    if counter == 0:
        return positions  # Original positions
    elif counter == 1:
        # Move to (x3, y3), (x1, y1), (x2, y2)
        return [positions[2], positions[0], positions[1]]
    elif counter == 2:
        # Move to (x2, y2), (x3, y3), (x1, y1)
        return [positions[1], positions[2], positions[0]]
    #Add ,pre condtions if there are more drones
    return positions  # Fallback



def rotate_triangle_around_center(positions, angle_degrees):
    
    """"
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


def calculate_rotation_params(offset_distance, set_speed):
    """
    Calculate the cycle time needed for a drone formation to complete a full rotation 
    around a user based on the offset distance and the set speed.

    :param offset_distance: The distance from the center point to each drone (radius), in meters.
    :param set_speed: The desired speed of the drones, in meters per second (m/s).
    :return: speed (in m/s), cycle_time (in seconds)
    """
    # Circumference of the circle traced by the drones
    circumference = 2 * math.pi * offset_distance  # in meters

    # Calculate the cycle time to complete a full rotation based on the set speed
    # Time = Distance / Speed
    cycle_time = circumference / set_speed  # in seconds

    return set_speed, cycle_time


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





def ensure_equal_distance(drones, triangle_positions, min_distance):
    # Ensure all drones maintain the minimum distance
    for i in range(len(drones)):
        for j in range(i + 1, len(drones)):
            drone1_pos = triangle_positions[i]
            drone2_pos = triangle_positions[j]
            distance = great_circle(drone1_pos, drone2_pos).meters

            print(f"Distance between {drones[i].id} and {drones[j].id}: {distance} meters")

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


def ensure_equal_distance_from_user(drones, triangle_positions, current_lat, current_lon, min_distance):
    # Ensure all drones maintain the minimum distance from the user's current location
    for i, drone_pos in enumerate(triangle_positions):
        distance_to_user = great_circle(drone_pos, (current_lat, current_lon)).meters
        print(f"Distance between {drones[i].id} and user: {distance_to_user} meters")

        if distance_to_user < min_distance:
            print(f"Adjusting position for {drones[i].id} to maintain distance from user")
            # Adjust position away from the user
            adjustment = (min_distance - distance_to_user)
            direction = np.array([drone_pos[0] - current_lat, drone_pos[1] - current_lon])
            direction_normalized = direction / np.linalg.norm(direction)  # Normalize direction
            
            triangle_positions[i] = (
                drone_pos[0] + adjustment * direction_normalized[0],
                drone_pos[1] + adjustment * direction_normalized[1]
            )
    
    return triangle_positions



def move_to_positions(drones, triangle_positions,kalman_user_speed, altitude):
    for drone, target_position in zip(drones, triangle_positions):
        drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'

        #drone.airspeed = kalman_user_speed
        #print(f"{drone_id}: Moving to triangle position at {target_position}...")

        drone.simple_goto(LocationGlobalRelative(target_position[0], target_position[1], altitude),kalman_user_speed)

        print(f"{drone_id}: Speed {drone.airspeed:.2f} m/s (Set Speed: {kalman_user_speed:.2f} m/s)")




"""
Controls drone via mavlink message
def move_to_positions(drones, triangle_positions, altitude):
    for drone, target_position in zip(drones, triangle_positions):
        drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'

        # Prepare the target position using LocationGlobalRelative
        target_location = LocationGlobalRelative(target_position[0], target_position[1], altitude)
            
        # Prepare and send the position target message
        target_msg = construct_position_target_message(drone, target_location)
            
        # Send the message to the drone
        drone.send_mavlink(target_msg)
        drone.flush()  # Ensure the message is sent immediately

        print(f"{drone_id}: Sending target position to {target_position}")

        time.sleep(0.1)  # Short delay for high-frequency position updates


def construct_position_target_message(vehicle, location):
    # Extract lat, lon, and alt from LocationGlobalRelative object
    lat = location.lat
    lon = location.lon
    alt = location.alt

    # Construct the MAVLink message for SET_POSITION_TARGET_GLOBAL_INT with adjusted type mask
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,                # time_boot_ms (not used)
        0, 0,             # target system and component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # coordinate frame (relative altitude)
        0b0000111111111000,  # type_mask (only position enabled)
        int(lat * 1e7),    # lat_int - Latitude in 1e7 * meters (integer)
        int(lon * 1e7),    # lon_int - Longitude in 1e7 * meters (integer)
        alt,               # altitude in meters (relative to home)
        0, 0, 0,           # X, Y, Z velocities in NED frame (not used)
        0, 0, 0,           # acceleration (not supported yet)
        0, 0)              # yaw and yaw_rate (not supported yet)
    
    return msg
"""


#Converts GPS Coordinates to position coordinates
"""
# Function to convert GPS coordinates (lat, lon, alt) to NED coordinates
def gps_to_ned(drone_lat, drone_lon, drone_alt, target_lat, target_lon, target_alt):
    # Constants for WGS-84 datum (Earth's radius in meters)
    R = 6378137.0  # Earth's radius in meters
    lat_diff = math.radians(target_lat - drone_lat)
    lon_diff = math.radians(target_lon - drone_lon)
    
    # Calculate differences in the NED frame
    north = lat_diff * R
    east = lon_diff * R * math.cos(math.radians(drone_lat))
    down = drone_alt - target_alt
    
    return north, east, down

# Function to move drones to their positions based on calculated GPS coordinates
def move_to_user_position(drones, user_lat, user_lon, user_alt, target_altitude, positions):

    #Moves each drone in `drones` to a position based on the formation pattern relative to the user's position.
    
    #Parameters:
    #- drones: list of drone objects
    #- user_lat, user_lon, user_alt: User's latitude, longitude, and altitude
    #- target_altitude: Target altitude relative to the user's altitude
    #- positions: List of drone target positions (lat, lon) for each drone

    for idx, drone in enumerate(drones):
        drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'

        # Get the drone's current GPS position
        current_lat = drone.location.global_frame.lat
        current_lon = drone.location.global_frame.lon
        current_alt = drone.location.global_frame.alt

        # Get the target GPS position for the drone from the positions list
        target_lat, target_lon = positions[idx]

        # Convert target GPS position to NED coordinates relative to the user's position
        north, east, down = gps_to_ned(user_lat, user_lon, user_alt, target_lat, target_lon, user_alt + target_altitude)

        # Create and send the NED position target message
        target_msg = construct_ned_target_message(drone, north, east, down)
        
        # Send the message to the drone
        drone.send_mavlink(target_msg)
        drone.flush()

        print(f"{drone_id}: Moving to target position (N:{north}, E:{east}, D:{down})")

        time.sleep(0.1)  # Short delay to ensure high-frequency updates

# Function to construct MAVLink message for NED coordinates
def construct_ned_target_message(vehicle, north, east, down):
    # Create MAVLink message for local NED positioning
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,                  # time_boot_ms (not used)
        0, 0,               # target system and component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # local NED frame
        0b0000111111111000,  # type_mask (only position enabled)
        north,               # Position in North (meters)
        east,                # Position in East (meters)
        down,                # Position in Down (altitude relative to home)
        0, 0, 0,             # velocities (unused)
        0, 0, 0,             # accelerations (unsupported)
        0, 0)                # yaw and yaw_rate (optional)

    return msg
    """

#Velocity based control

def gps_to_ned_velocity(drone_lat, drone_lon, target_lat, target_lon, user_speed):
    # Constants for Earth's radius in meters (WGS-84)
    R = 6378137.0

    # Calculate differences in latitude and longitude in radians
    lat_diff = math.radians(target_lat - drone_lat)
    lon_diff = math.radians(target_lon - drone_lon)

    # Calculate the NED distances
    north = lat_diff * R
    east = lon_diff * R * math.cos(math.radians(drone_lat))

    # Normalize the direction and scale by the user's speed
    distance = math.sqrt(north**2 + east**2)
    if distance > 0:
        north_velocity = (north / distance) * user_speed
        east_velocity = (east / distance) * user_speed
    else:
        north_velocity = east_velocity = 0

    return north_velocity, east_velocity



def move_to_positions_velocity(drones, triangle_positions, kalman_user_speed, target_altitude, alpha=0.3):
    for drone, target_position in zip(drones, triangle_positions):
        drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'

        # Get the drone's current GPS position and altitude
        current_lat = drone.location.global_relative_frame.lat
        current_lon = drone.location.global_relative_frame.lon
        current_alt = drone.location.global_relative_frame.alt  # Use relative frame for altitude
        previous_alt = getattr(drone, 'previous_alt', current_alt)

        # Calculate the NED horizontal velocity to reach the target position
        raw_north_velocity, raw_east_velocity = gps_to_ned_velocity(
            current_lat, current_lon,
            target_position[0], target_position[1],
            kalman_user_speed
        )

        # Apply a low-pass filter to north and east velocities for smoothing
        filtered_north_velocity = (alpha * raw_north_velocity +
                                   (1 - alpha) * getattr(drone, 'prev_north_velocity', 0))
        filtered_east_velocity = (alpha * raw_east_velocity +
                                  (1 - alpha) * getattr(drone, 'prev_east_velocity', 0))

        # Update previous velocities for filtering in the next iteration
        setattr(drone, 'prev_north_velocity', filtered_north_velocity)
        setattr(drone, 'prev_east_velocity', filtered_east_velocity)

        # Altitude error and rate of change (PD control)
        altitude_error = target_altitude - current_alt
        altitude_rate = current_alt - previous_alt  # Change in altitude since last iteration

        # PD Control for altitude
        kp = 0.5  # Proportional gain
        kd = 0.1  # Derivative gain
        pd_down_velocity = kp * altitude_error - kd * altitude_rate

        # Apply ascending/descending logic for larger corrections
        if abs(altitude_error) > 0.1:  # If altitude error exceeds tolerance
            if altitude_error > 0 and pd_down_velocity > -0.5:  # Ascend if target is higher
                pd_down_velocity = max(pd_down_velocity, -0.5)  # Cap descent rate for stability
            elif altitude_error < 0 and pd_down_velocity < 0.5:  # Descend if target is lower
                pd_down_velocity = min(pd_down_velocity, 0.5)  # Cap ascent rate for stability
        else:
            pd_down_velocity = 0.0  # Maintain altitude when close to target

        # Store current altitude for use in the next iteration
        setattr(drone, 'previous_alt', current_alt)

        # Construct and send MAVLink velocity command for NED movement
        send_ned_velocity(drone, filtered_north_velocity, filtered_east_velocity, pd_down_velocity)
        
        print(f"{drone_id}: Moving towards target with velocities N: {filtered_north_velocity:.2f} m/s, "
              f"E: {filtered_east_velocity:.2f} m/s, D: {pd_down_velocity:.2f} m/s")

        # Optional: Short delay for smoother control loop (e.g., 100 ms)
        time.sleep(0.1)





def send_ned_velocity(vehicle, north, east, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system and target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Frame of reference
        0b0000111111000111,  # Type mask (only velocities enabled)
        0, 0, 0,             # x, y, z positions (not used here)
        north, east, down,   # x, y, z velocity in m/s
        0, 0, 0,             # x, y, z acceleration (not supported)
        0, 0)                # yaw, yaw_rate (not supported)
    vehicle.send_mavlink(msg)
    vehicle.flush()



def wait_for_drones_to_reach_positions(drones, triangle_positions, stop_operations_event):
    """
    Waits for all drones to reach their specified positions in a triangle formation.

    Args:
        drones: A list of drone objects.
        triangle_positions: A list of tuples representing the target positions for each drone.
        stop_operations_event: An event object to signal a graceful shutdown.
    """
    for i, drone in enumerate(drones):
        target_position = triangle_positions[i]
        drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'

        # Wait until the drone reaches its target position
        while True:
            if stop_operations_event.is_set():
                print(f"{drone_id}: Stop signal received. Aborting movement.")
                break
            current_position = (drone.location.global_relative_frame.lat, drone.location.global_relative_frame.lon)
            distance_to_target = great_circle(current_position, target_position).meters
            #print(f"{drone_id}: Distance to target: {distance_to_target:.2f} meters")

            # Check if the drone has reached its target position with a tolerance (e.g., 1 meter)
            if distance_to_target < 1:
                print(f"{drone_id}: Reached target position.")
                break

            #time.sleep(0.5)  # Adjust the sleep time as needed

def read_accelerometer_data(stationary=False):
    """Simulate reading data from the accelerometer."""
    noise = random.uniform(-0.01, 0.01)  # Reduced noise
    if stationary:
        ax = ay = 0.0 + noise  # No acceleration when stationary
    else:
        ax = random.uniform(-0.2, 0.2) + noise  # Reduced range for more realistic motion
        ay = random.uniform(-0.2, 0.2) + noise  # Reduced range for more realistic motion
    az = random.uniform(-0.01, 0.01)  # Minimal vertical movement noise
    return ax, ay, az


def read_gyroscope_data(stationary=False):
    """Simulate reading data from the gyroscope."""
    noise = random.uniform(-0.1, 0.1)  # Reduced noise
    if stationary:
        gx = gy = gz = 0.0 + noise  # Minimal drift when stationary
    else:
        gx = random.uniform(-1.0, 1.0) + noise
        gy = random.uniform(-1.0, 1.0) + noise
        gz = random.uniform(-1.0, 1.0) + noise
    return gx, gy, gz


def read_gps_data(reference_lat, reference_lon, stationary=False):
    """Simulate a GPS reading with or without user movement."""
    if stationary:
        # Simulate very minimal noise when stationary
        gps_lat_noise = random.uniform(-0.0000001, 0.0000001)  # Very small noise
        gps_lon_noise = random.uniform(-0.0000001, 0.0000001)
    else:
        # Simulate normal GPS noise
        gps_lat_noise = random.uniform(-0.000005, 0.000005)
        gps_lon_noise = random.uniform(-0.000005, 0.000005)

    gps_lat = reference_lat + gps_lat_noise
    gps_lon = reference_lon + gps_lon_noise

    return gps_lat, gps_lon



def read_compass_data():
    """Simulate reading data from a compass (magnetometer) with noise."""
    noise = random.uniform(-2, 2)  # Simulated noise
    heading = (random.uniform(0, 360) + noise) % 360
    return heading



def kalman_filter(state, P, measurement, accel, gyro, compass, dt, stationary=False):
    """
    Perform one iteration of the Kalman filter with extended data.
    """
    # State transition matrix (A)
    A = np.array([
        [1, 0, dt, 0],  # Update latitude based on velocity_lat
        [0, 1, 0, dt],  # Update longitude based on velocity_lon
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

    # Adjust process noise covariance (Q) based on stationary state
    process_noise = 0.0001 if stationary else 0.001
    Q = np.eye(4) * process_noise

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

    # Adjust measurement noise covariance (R) based on stationary state
    measurement_noise = 0.0001 if stationary else 0.005
    R = np.eye(2) * measurement_noise

    # Innovation or measurement residual (Y)
    Y = Z - H @ state

    # Innovation covariance (S)
    S = H @ P @ H.T + R

    # Kalman gain (K)
    K = P @ H.T @ np.linalg.inv(S)

    # Update state estimate and covariance
    state = state + K @ Y
    P = (np.eye(4) - K @ H) @ P

    # Apply a dead zone for very small velocities (e.g., <0.1 m/s)
    state[2] = 0.0 if abs(state[2]) < 0.1 else state[2]  # Velocity_lat
    state[3] = 0.0 if abs(state[3]) < 0.1 else state[3]  # Velocity_lon

    return state, P




def simulate_user_movement(reference_lat, reference_lon, pause_duration=0.5, stationary=False):
    """Simulate user movement or stationary state."""
    # Initial state: [latitude, longitude, velocity_lat, velocity_lon]
    state = np.array([reference_lat, reference_lon, 0, 0])
    P = np.eye(4)  # Initial uncertainty

    # Time step (in seconds)
    dt = pause_duration

    # Read accelerometer, gyroscope, GPS, and compass data
    gps_lat, gps_lon = read_gps_data(reference_lat, reference_lon, stationary)
    ax, ay, az = read_accelerometer_data(stationary)
    gx, gy, gz = read_gyroscope_data(stationary)
    heading = read_compass_data()

    # Update the state using the Kalman filter
    state, P = kalman_filter(state, P, (gps_lat, gps_lon), (ax, ay), (gx, gy, gz), heading, dt, stationary=stationary)

    # Extract updated latitude and longitude from the state
    new_lat = state[0]
    new_lon = state[1]

    # Extract speed from Kalman filter state (state[2] and state[3] are velocities)
    kalman_speed_x = state[2]
    kalman_speed_y = state[3]

    # Output Kalman speed (m/s)
    kalman_user_speed = np.sqrt(kalman_speed_x**2 + kalman_speed_y**2)
    print(f"Kalman Speed (User): {kalman_user_speed:.2f} m/s")

    # Pause for a specified duration between movements
    time.sleep(pause_duration)

    return new_lat, new_lon, kalman_user_speed

def determine_user_coordinates(current_lat, current_lon, user_speed, last_known_lat=None, last_known_lon=None, is_stationary=False, stationary_speed_threshold=0.5):
    """
    Determine the user's coordinates based on movement.

    Args:
        current_lat (float): Current latitude of the user.
        current_lon (float): Current longitude of the user.
        user_speed (float): Current speed of the user.
        last_known_lat (float): Last known latitude when the user was stationary.
        last_known_lon (float): Last known longitude when the user was stationary.
        is_stationary (bool): Whether the user was previously stationary.
        stationary_speed_threshold (float): Speed threshold to determine if the user is stationary.

    Returns:
        tuple: (user_orbit_lat, user_orbit_lon, last_known_lat, last_known_lon, is_stationary)
    """
    if user_speed < stationary_speed_threshold:  # User is stationary
        if not is_stationary:
            # Cache the current position
            last_known_lat, last_known_lon = current_lat, current_lon
            print("User is stationary. Caching last known GPS position.")
            is_stationary = True
    else:
        is_stationary = False  # User is moving

    # Use last known position if stationary, otherwise live GPS data
    user_orbit_lat = last_known_lat if is_stationary else current_lat
    user_orbit_lon = last_known_lon if is_stationary else current_lon

    return user_orbit_lat, user_orbit_lon, last_known_lat, last_known_lon, is_stationary




def operate_drones(drones, target_altitude, reference_lat, reference_lon, offset_distance, orbit_around_user, swap_positions, swap_position_speed, rotate_triangle_formation,angle_offset,revolve_speed,revolve_offset_distance):
    global stop_operations_event  # Use the global stop flag
    

    # Arm and take off each drone
    threads = []
    for drone in drones:
        drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'
        thread = threading.Thread(target=arm_and_takeoff, args=(drone, target_altitude, drone_id, stop_operations_event))
        thread.start()
        threads.append(thread)

    # Wait for all arming and takeoff threads to finish
    for thread in threads:
        thread.join()

    # Allow a moment for all drones to stabilize after the takeoff command

    print("Moving to positions")
    kalman_user_speed = 10
    triangle_positions = calculate_triangle_positions(reference_lat, reference_lon, offset_distance)
    move_to_positions(drones, triangle_positions, kalman_user_speed,target_altitude)
    wait_for_drones_to_reach_positions(drones, triangle_positions, stop_operations_event)


    time.sleep(4)


 
    previous_time = time.time()
    counter = 0

    last_known_lat, last_known_lon = None, None  # Initialize last known coordinates
    is_stationary = False  # Initialize stationary flag

 
    try:
        # Main loop: move drones based on simulated user movement
        while not stop_operations_event.is_set():
            # Simulate user movement (you would replace this with real GPS data for the user)
            current_lat, current_lon, kalman_user_speed = simulate_user_movement(reference_lat, reference_lon, pause_duration=0.5,stationary=True)
            #current_lat, current_lon = read_gps_data(reference_lat, reference_lon, stationary=True)

            #current_lat = reference_lat
            #current_lon = reference_lon
            #kalman_user_speed=0.4

            #time.sleep(1)


            user_orbit_lat, user_orbit_lon, last_known_lat, last_known_lon, is_stationary = determine_user_coordinates(
                current_lat, current_lon, kalman_user_speed, last_known_lat, last_known_lon, is_stationary
            )


            #print("latitude:" + str(user_orbit_lat)  + " " + "longitude:" + "" + str(user_orbit_lon))
            #is_stationary = kalman_user_speed < 0.1  # Threshold for stationary
        
            if is_stationary:
                if orbit_around_user:
                    current_time = time.time()
                    elapsed_time = current_time - previous_time
                    previous_time = current_time


                    # Calculate speed and cycle time based on current parameters
                    speed, cycle_time = calculate_rotation_params(revolve_offset_distance, revolve_speed)

                    # Adjust the angle_offset based on speed and elapsed time
                    angle_offset += speed * elapsed_time


                    triangle_positions = calculate_revolving_positions(
                        user_orbit_lat, user_orbit_lon, revolve_offset_distance, len(drones), angle_offset)
                    #triangle_positions = ensure_equal_distance_from_user(drones, triangle_positions, current_lat, current_lon, offset_distance)

                    #print("Orbiting Positions:", triangle_positions)

                    move_to_positions(drones, triangle_positions, speed, target_altitude)
                    #move_to_positions_velocity(drones, triangle_positions, kalman_user_speed, target_altitude)

    
                    print(f"Cycle Time: {cycle_time:.2f} seconds, Speed: {speed:.2f} m/s")
                    


                elif rotate_triangle_formation:
                    current_time = time.time()
                    elapsed_time = current_time - previous_time
                    previous_time = current_time


                    # Calculate speed and cycle time based on current parameters
                    speed, cycle_time = calculate_rotation_params(revolve_offset_distance, revolve_speed)

                    # Adjust the angle_offset based on speed and elapsed time
                    angle_offset += speed * elapsed_time
               
                    triangle_positions = calculate_triangle_positions(user_orbit_lat, user_orbit_lon, revolve_offset_distance)
                    triangle_positions = rotate_triangle_around_center(triangle_positions, angle_offset)
                
                    #print("Rotating Positions:", triangle_positions)
                
                    move_to_positions(drones, triangle_positions, speed, target_altitude)
                    #move_to_positions_velocity(drones, triangle_positions, speed, target_altitude)

                    print(f"Cycle Time: {cycle_time:.2f} seconds, Speed: {speed:.2f} m/s")
      

                elif swap_positions:
        
                    triangle_positions = calculate_triangle_positions(user_orbit_lat, user_orbit_lon, revolve_offset_distance)
                    #triangle_positions = ensure_equal_distance(drones, triangle_positions, offset_distance)
                    #triangle_positions = ensure_equal_distance_from_user(drones, triangle_positions, current_lat, current_lon, offset_distance)


                    triangle_positions = swap_triangle_positions(triangle_positions, counter)
                    #print(triangle_positions)

                    # Increment the counter and reset if necessary
                    counter += 1
                    if counter > 2:  # Reset after maximum number of swaps
                        counter = 0
              
                    #triangle_positions = ensure_equal_distance(drones, triangle_positions, offset_distance)
                    #triangle_positions = ensure_equal_distance_from_user(drones, triangle_positions, current_lat, current_lon, offset_distance)

                    move_to_positions(drones, triangle_positions, kalman_user_speed, target_altitude)
                    #move_to_positions_velocity(drones, triangle_positions, kalman_user_speed, target_altitude)

                    # Wait for all drones to reach their current positions
                    wait_for_drones_to_reach_positions(drones, triangle_positions, stop_operations_event)

                

                    #time_needed = offset_distance / kalman_user_speed
                    #time.sleep(time_needed)
                    #print(time_needed)

                    #time.sleep(3.0)


   
                else:
                    # Calculate and adjust positions for triangular formation
                    triangle_positions = calculate_triangle_positions(user_orbit_lat, user_orbit_lon, revolve_offset_distance)
                    triangle_positions = ensure_equal_distance(drones, triangle_positions, offset_distance)
                    #triangle_positions = ensure_equal_distance_from_user(drones, triangle_positions, current_lat, current_lon, offset_distance)

                    move_to_positions(drones, triangle_positions, kalman_user_speed, target_altitude)
                    #move_to_positions_velocity(drones, triangle_positions, kalman_user_speed, target_altitude)
                    print("User is moving")
  
        
                    #print(triangle_positions)

                    # Move drones to the calculated positions
                    #move_to_positions(drones, triangle_positions, kalman_user_speed, target_altitude)
    

        
                    # Monitor drones for issues (battery, GPS, etc.)
                    if monitor_drones(drones, low_battery_threshold=20, stop_operations_event=stop_operations_event):
                        break  # Stop operations if monitoring indicates issues
            
                    # Short sleep to give time for drones to adjust
                    #time.sleep(1)

    except (KeyboardInterrupt, TimeoutError, ValueError, Exception) as e:
        handle_drone_exceptions(e, stop_operations_event)

    finally:
        # Ensure that the drones land regardless of the reason for stopping
        for drone in drones:
            drone_id = drone.id if hasattr(drone, 'id') else 'Unknown'
            try:
                land(drone, drone_id)
            except Exception as e:
                print(f"Error during landing of {drone_id}: {e}")

        print("Drones have landed safely.")

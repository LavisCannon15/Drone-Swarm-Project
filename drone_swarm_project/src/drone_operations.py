from dronekit import VehicleMode
import time
import sys

def arm_and_takeoff(vehicle, target_altitude, hover_time=5, drone_id="Drone"):
    """
    Arms the drone, flies to the target altitude, and hovers for a specified time.
    :param vehicle: The connected Vehicle object.
    :param target_altitude: Target altitude in meters.
    :param hover_time: Time to hover at the target altitude in seconds.
    :param drone_id: Identifier for the drone (for printing purposes).
    """
    print(f"{drone_id}: Basic pre-arm checks")
    while not vehicle.is_armable:
        print(f"{drone_id}: Waiting for vehicle to initialize...", end='\r')
        time.sleep(1)

    print(f"{drone_id}: Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(f"{drone_id}: Waiting for arming...", end='\r')
        time.sleep(1)

    print(f"{drone_id}: Taking off! Target altitude: {target_altitude} meters")
    vehicle.simple_takeoff(target_altitude)

    # Wait until the vehicle reaches a safe height before proceeding
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        print(f"{drone_id}: Altitude: {altitude:.2f} meters", end='\r')
        if altitude >= target_altitude * 0.95:
            print(f"{drone_id}: Reached target altitude")
            break
        time.sleep(1)

    # Hover for the specified hover_time
    print(f"{drone_id}: Hovering for {hover_time} seconds")
    time.sleep(hover_time)

def land(vehicle, drone_id="Drone"):
    """
    Commands the drone to land.
    :param vehicle: The connected Vehicle object.
    :param drone_id: Identifier for the drone (for printing purposes).
    """
    print(f"{drone_id}: Landing...")
    vehicle.mode = VehicleMode("LAND")

    while vehicle.armed:
        altitude = vehicle.location.global_relative_frame.alt
        print(f"{drone_id}: Altitude during landing: {altitude:.2f} meters", end='\r')
        time.sleep(1)

    print(f"{drone_id}: Landed and disarmed")

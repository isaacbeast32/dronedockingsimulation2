from pymavlink import mavutil
import time
import sys

def connect_drone(name, address, timeout=20):
    print(f"Connecting to {name} on {address}...")
    drone = mavutil.mavlink_connection(address)
    hb = drone.wait_heartbeat(timeout=timeout)
    if hb is None:
        raise TimeoutError(f"Timed out waiting for heartbeat from {name} on {address}")
    print(f"{name} connected")
    return drone

def set_mode(drone, mode):
    drone.set_mode_apm(mode)
    time.sleep(1)

def arm_and_takeoff(drone, altitude):
    print(f"Arming and taking off to {altitude}m")
    drone.arducopter_arm()
    time.sleep(2)

    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    time.sleep(8)

def send_position(drone, x, y, z):
    drone.mav.set_position_target_local_ned_send(
        0,
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111111000,
        x, y, -z,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

try:
    drone1 = connect_drone("Drone 1", "udp:127.0.0.1:14550")
    drone2 = connect_drone("Drone 2", "udp:127.0.0.1:14560")

    set_mode(drone1, "GUIDED")
    arm_and_takeoff(drone1, 3)
    send_position(drone1, 0, 0, 3)

    set_mode(drone2, "GUIDED")
    arm_and_takeoff(drone2, 6)

    send_position(drone2, 0, 0, 6)
    time.sleep(5)

    print("Drone 2 descending...")
    for alt in [5, 4, 3.5, 3.3, 3.2, 3.1]:
        send_position(drone2, 0, 0, alt)
        print(f"Altitude: {alt}")
        time.sleep(3)

    print("Docking attempt complete")

except Exception as e:
    print(f"Error: {e}")
    sys.exit(1)

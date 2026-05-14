from pymavlink import mavutil
import time
import threading
import subprocess
import math

# ── Config ────────────────────────────────────────────────────────────────────
ATTACH_TOPIC  = "/model/drone1/detachable_joint/attach"
DETACH_TOPIC  = "/model/drone1/detachable_joint/detach"
STATE_TOPIC   = "/model/drone2::iris_with_standoffs_female/detachable_joint/state"

DOCK_ALT      = 1.60
APPROACH_STEP = 0.04
STEP_DWELL    = 0.15
STOP_GAP      = 0.10
SETPOINT_HZ   = 10


# ── Connection ────────────────────────────────────────────────────────────────

def connect(name, conn_str):
    print(f"[{name}] Connecting...")
    d = mavutil.mavlink_connection(conn_str)
    d.wait_heartbeat()
    print(f"[{name}] Connected")
    return d


def set_mode(drone, mode, name=""):
    drone.mav.set_mode_send(
        drone.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        drone.mode_mapping()[mode]
    )
    time.sleep(2)
    print(f"[{name}] Mode → {mode}")


def arm_and_takeoff(drone, altitude, name="", arm_retries=3):
    for attempt in range(1, arm_retries + 1):
        print(f"[{name}] Arming (attempt {attempt}/{arm_retries})...")
        drone.arducopter_arm()
        deadline = time.time() + 12
        armed = False
        while time.time() < deadline:
            msg = drone.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if msg and (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                armed = True
                print(f"[{name}] Armed ✓")
                break
        if armed:
            break
        if attempt < arm_retries:
            print(f"[{name}] Arm timed out — resetting in 3s...")
            drone.arducopter_disarm()
            time.sleep(3)
            set_mode(drone, "GUIDED", name)
        else:
            print(f"[{name}] ERROR: Could not arm after {arm_retries} attempts")
            return False

    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    print(f"[{name}] Takeoff command sent...")
    return True


def wait_for_altitude(drone, target_alt, name="", tolerance=0.15, timeout=30.0):
    drone.mav.request_data_stream_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1
    )
    print(f"[{name}] Climbing to {target_alt}m...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = drone.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2.0)
        if msg:
            current_alt = -msg.z
            print(f"[{name}] alt={current_alt:.2f}m", end="\r")
            if current_alt >= (target_alt - tolerance):
                print(f"\n[{name}] Reached {current_alt:.2f}m ✓")
                return True
    print(f"\n[{name}] ERROR: Did not reach altitude in {timeout}s")
    return False


# ── Position + attitude ───────────────────────────────────────────────────────

def request_streams(drone, name=""):
    for stream in [mavutil.mavlink.MAV_DATA_STREAM_POSITION,
                   mavutil.mavlink.MAV_DATA_STREAM_EXTRA1]:
        drone.mav.request_data_stream_send(
            drone.target_system, drone.target_component,
            stream, 10, 1
        )
    time.sleep(0.5)
    print(f"[{name}] Streams requested")


def flush(drone, msg_type, duration=0.5):
    t = time.time()
    while time.time() - t < duration:
        drone.recv_match(type=msg_type, blocking=False)


def get_position(drone, name="", timeout=10.0):
    flush(drone, "LOCAL_POSITION_NED")
    deadline = time.time() + timeout
    attempt = 0
    while time.time() < deadline:
        msg = drone.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2.0)
        attempt += 1
        if msg:
            x, y, z = msg.x, msg.y, -msg.z
            print(f"[{name}] pos  x={x:.3f}  y={y:.3f}  z={z:.3f}")
            return x, y, z
        print(f"[{name}] Waiting for position... ({attempt})")
    print(f"[{name}] ERROR: No position received")
    return None


def get_yaw(drone, name="", timeout=10.0):
    flush(drone, "ATTITUDE")
    deadline = time.time() + timeout
    attempt = 0
    while time.time() < deadline:
        msg = drone.recv_match(type="ATTITUDE", blocking=True, timeout=2.0)
        attempt += 1
        if msg:
            yaw_rad = msg.yaw
            print(f"[{name}] yaw  {math.degrees(yaw_rad):.1f}°")
            return yaw_rad
        print(f"[{name}] Waiting for attitude... ({attempt})")
    print(f"[{name}] WARNING: Could not read yaw — defaulting to 0")
    return 0.0


# ── Gazebo ────────────────────────────────────────────────────────────────────

def check_attached():
    try:
        result = subprocess.run(
            ["gz", "topic", "-e", "-n", "1", "-t", STATE_TOPIC],
            capture_output=True, text=True, timeout=3
        )
        if "is_attached: true" in result.stdout:
            return True
        if "is_attached: false" in result.stdout:
            return False
    except subprocess.TimeoutExpired:
        pass
    return None


def gz(topic):
    subprocess.run(
        ["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", ""],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
    )


def force_detach_on_startup():
    print("Sending startup detach (ensures clean state)...")
    for _ in range(5):
        gz(DETACH_TOPIC)
        time.sleep(0.2)
    time.sleep(0.5)
    state = check_attached()
    if state is True:
        print("✗ Still attached — restart Gazebo.")
        return False
    print("✓ Joint confirmed detached")
    return True


def attach_until_latched(stop_event, interval=0.2):
    while not stop_event.is_set():
        gz(ATTACH_TOPIC)
        time.sleep(interval)


# ── Movement ──────────────────────────────────────────────────────────────────

def setpoint(drone, x, y, z, yaw):
    drone.mav.set_position_target_local_ned_send(
        0,
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000_1111_1111_1000,
        x, y, -z,
        0, 0, 0,
        0, 0, 0,
        yaw, 0.0
    )


def hold(drone, x, y, z, yaw, seconds):
    end = time.time() + seconds
    while time.time() < end:
        setpoint(drone, x, y, z, yaw)
        time.sleep(1.0 / SETPOINT_HZ)


def hold_both(d1, p1, yaw1, d2, p2, yaw2, seconds):
    t1 = threading.Thread(target=hold, args=(d1, *p1, yaw1, seconds))
    t2 = threading.Thread(target=hold, args=(d2, *p2, yaw2, seconds))
    t1.start(); t2.start()
    t1.join();  t2.join()


def fly_to(drone, tx, ty, tz, yaw, name="", tolerance=0.15, timeout=20.0):
    """
    Command drone to (tx, ty, tz) and block until it arrives.
    Keeps sending setpoints at SETPOINT_HZ while waiting.
    """
    print(f"[{name}] Flying to x={tx:.2f} y={ty:.2f} z={tz:.2f}...")
    deadline = time.time() + timeout

    # Request position stream so we can read progress
    drone.mav.request_data_stream_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 10, 1
    )

    while time.time() < deadline:
        setpoint(drone, tx, ty, tz, yaw)

        msg = drone.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=0.1)
        if msg:
            cx, cy, cz = msg.x, msg.y, -msg.z
            dist = math.sqrt((cx - tx)**2 + (cy - ty)**2 + (cz - tz)**2)
            print(f"[{name}] → dist={dist:.2f}m", end="\r")
            if dist <= tolerance:
                print(f"\n[{name}] Reached target ✓")
                return True

    print(f"\n[{name}] WARNING: Did not reach target in {timeout}s — continuing")
    return False


def gentle_land(drone, x, y, yaw, name=""):
    """
    Descend in steps from DOCK_ALT to near ground, then switch to LAND.
    Avoids the sudden throttle cut that makes drones fall out of the sky.
    """
    print(f"[{name}] Gentle descent...")
    for alt in [1.2, 0.8, 0.4, 0.2]:
        hold(drone, x, y, alt, yaw, seconds=1.5)
    print(f"[{name}] Switching to LAND...")
    set_mode(drone, "LAND", name)


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print("=" * 55)
    print("  Docking: Drone1 moves along Y into stationary Drone2")
    print("=" * 55)

    # Verify attach topic
    print("\nChecking Gazebo topics...")
    result = subprocess.run(["gz", "topic", "--list"], capture_output=True, text=True)
    if ATTACH_TOPIC not in result.stdout:
        print(f"✗ Attach topic missing: {ATTACH_TOPIC}")
        print("  Fix your SDF plugin and restart Gazebo.")
        return
    print(f"✓ {ATTACH_TOPIC}")

    if not force_detach_on_startup():
        return

    # Connect
    drone1 = connect("Drone1", "tcp:127.0.0.1:5760")
    drone2 = connect("Drone2", "tcp:127.0.0.1:5770")

    # Takeoff
    set_mode(drone1, "GUIDED", "Drone1")
    if not arm_and_takeoff(drone1, DOCK_ALT, "Drone1"):
        return
    if not wait_for_altitude(drone1, DOCK_ALT, "Drone1"):
        set_mode(drone1, "LAND", "Drone1")
        return

    set_mode(drone2, "GUIDED", "Drone2")
    if not arm_and_takeoff(drone2, DOCK_ALT, "Drone2"):
        set_mode(drone1, "LAND", "Drone1")
        return
    if not wait_for_altitude(drone2, DOCK_ALT, "Drone2"):
        set_mode(drone1, "LAND", "Drone1")
        set_mode(drone2, "LAND", "Drone2")
        return

    # Read positions + yaws — these become the HOME positions to return to
    print("\nReading positions and headings...")
    request_streams(drone1, "Drone1")
    request_streams(drone2, "Drone2")
    time.sleep(1)

    p1 = get_position(drone1, "Drone1")
    p2 = get_position(drone2, "Drone2")
    yaw1 = get_yaw(drone1, "Drone1")
    yaw2 = get_yaw(drone2, "Drone2")

    if p1 is None or p2 is None:
        print("ERROR: Could not read positions. Landing.")
        set_mode(drone1, "LAND", "Drone1")
        set_mode(drone2, "LAND", "Drone2")
        return

    # Save spawn positions — used for return after detach
    home1 = (p1[0], p1[1])   # (x, y) at hover after takeoff
    home2 = (p2[0], p2[1])

    d2x, d2y = p2[0], p2[1]
    d1x      = p1[0]
    d1y      = p1[1]

    dy        = d2y - d1y
    direction = 1.0 if dy > 0 else -1.0
    target_y  = d2y - direction * STOP_GAP

    print(f"\nDrone1 home: x={home1[0]:.3f}  y={home1[1]:.3f}  yaw={math.degrees(yaw1):.1f}°")
    print(f"Drone2 home: x={home2[0]:.3f}  y={home2[1]:.3f}  yaw={math.degrees(yaw2):.1f}°")
    print(f"Drone1 target y={target_y:.3f}  travel={abs(target_y - d1y):.3f}m\n")

    # Stabilise
    print("Stabilising (4s)...")
    hold_both(
        drone1, (d1x, d1y, DOCK_ALT), yaw1,
        drone2, (d2x, d2y, DOCK_ALT), yaw2,
        seconds=4
    )

    # Approach
    print("Drone1 approaching...")
    current_y = d1y
    steps = int(abs(target_y - current_y) / APPROACH_STEP)

    for _ in range(steps):
        current_y += direction * APPROACH_STEP
        print(f"  Drone1 y={current_y:.3f}  →  {target_y:.3f}", end="\r")
        t1 = threading.Thread(target=hold,
             args=(drone1, d1x, current_y, DOCK_ALT, yaw1, STEP_DWELL))
        t2 = threading.Thread(target=hold,
             args=(drone2, d2x, d2y,       DOCK_ALT, yaw2, STEP_DWELL))
        t1.start(); t2.start()
        t1.join();  t2.join()

    current_y = target_y
    print(f"\nDrone1 at y={current_y:.3f} — pre-latch hold (3s)...")
    hold_both(
        drone1, (d1x, current_y, DOCK_ALT), yaw1,
        drone2, (d2x, d2y,       DOCK_ALT), yaw2,
        seconds=3
    )

    # Latch
    print("\nLatching...")
    stop_ev = threading.Event()
    attach_t = threading.Thread(target=attach_until_latched, args=(stop_ev,))
    attach_t.start()

    latched = False
    deadline = time.time() + 10
    while time.time() < deadline:
        hold_both(
            drone1, (d1x, current_y, DOCK_ALT), yaw1,
            drone2, (d2x, d2y,       DOCK_ALT), yaw2,
            seconds=0.5
        )
        if check_attached() is True:
            latched = True
            break
        print("  Waiting for latch...", end="\r")

    stop_ev.set()
    attach_t.join()

    if not latched:
        print("⚠ Could not confirm latch — continuing")
    else:
        print("✓ LATCHED")

    # Stop setpoints — let ArduPilot hold internally
    print("\nSetpoints stopped — ArduPilot holding.")

    print("\n" + "=" * 55)
    print("  DOCKED")
    print("=" * 55)
    print(f'\nTo detach: gz topic -t {DETACH_TOPIC} -m gz.msgs.Empty -p ""')
    print("Press ENTER to detach and return home...")
    input()

    # ── Detach ────────────────────────────────────────────────────
    print("Detaching...")
    gz(DETACH_TOPIC)
    time.sleep(2)   # let physics separate before issuing movement

    # ── Return to home positions simultaneously ───────────────────
    print("\nReturning to home positions...")
    t1 = threading.Thread(
        target=fly_to,
        args=(drone1, home1[0], home1[1], DOCK_ALT, yaw1),
        kwargs={"name": "Drone1"}
    )
    t2 = threading.Thread(
        target=fly_to,
        args=(drone2, home2[0], home2[1], DOCK_ALT, yaw2),
        kwargs={"name": "Drone2"}
    )
    t1.start(); t2.start()
    t1.join();  t2.join()

    print("\nBoth drones at home — holding 3s...")
    hold_both(
        drone1, (home1[0], home1[1], DOCK_ALT), yaw1,
        drone2, (home2[0], home2[1], DOCK_ALT), yaw2,
        seconds=3
    )

    # ── Gentle land — step down then LAND mode ────────────────────
    # Stagger slightly so prop wash doesn't interact
    print("\nLanding...")
    t1 = threading.Thread(
        target=gentle_land,
        args=(drone1, home1[0], home1[1], yaw1),
        kwargs={"name": "Drone1"}
    )
    t2 = threading.Thread(
        target=gentle_land,
        args=(drone2, home2[0], home2[1], yaw2),
        kwargs={"name": "Drone2"}
    )
    t1.start()
    time.sleep(0.5)   # slight stagger
    t2.start()
    t1.join(); t2.join()

    print("Done.")


if __name__ == "__main__":
    main()

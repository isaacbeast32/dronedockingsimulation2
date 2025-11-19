#!/usr/bin/env bash
# -------------------------------------------------------------
# ArduPilot + Gazebo Harmonic auto-launch script
# Works on Ubuntu 24.04 WSL 2
# -------------------------------------------------------------

# --- Configuration ---
ARDUPILOT_DIR="$HOME/ardupilot"
GAZEBO_DIR="$HOME/ardupilot_gazebo"
WORLD="$GAZEBO_DIR/worlds/iris_runway.sdf"

# --- Environment for Gazebo Harmonic ---
export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH="$GAZEBO_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="$GAZEBO_DIR/models:$GAZEBO_DIR/worlds:${GZ_SIM_RESOURCE_PATH}"

# --- Check folders ---
if [ ! -f "$WORLD" ]; then
  echo "[ERROR]  World file not found: $WORLD"
  exit 1
fi
if [ ! -f "$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" ]; then
  echo "[ERROR]  ArduPilot not found in $ARDUPILOT_DIR"
  exit 1
fi

# --- Launch Gazebo in background ---
echo "[INFO]  Launching Gazebo Harmonic..."
cd "$GAZEBO_DIR"
gz sim -r "$WORLD" -v4 &
GAZEBO_PID=$!
sleep 5  # give Gazebo time to load

# --- Launch ArduPilot SITL ---
echo "[INFO]  Launching ArduPilot SITL (Copter) and connecting to Gazebo..."
cd "$ARDUPILOT_DIR"
python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console

# --- When SITL exits, clean up Gazebo ---
echo "[INFO]  Shutting down Gazebo..."
kill $GAZEBO_PID 2>/dev/null
echo "[INFO]  Simulation session finished."

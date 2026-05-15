#!/bin/bash
export DISABLE_ROS1_EOL_WARNINGS=1
cd /home/hashie/catkin_ws
catkin_make
python3 src/com760cw2_group6/scripts/generate_world.py
source src/com760cw2_group6/worlds/spawn_config.env

# Launch Gazebo in background so we can start the fire oscillator once it's ready
roslaunch com760cw2_group6 group_6_launch.launch > /tmp/demo_gazebo.log 2>&1 &
LAUNCH_PID=$!

cleanup() {
    echo ""
    echo "[demo] Shutting down..."
    kill "$LAUNCH_PID" 2>/dev/null
    wait "$LAUNCH_PID" 2>/dev/null
    exit 0
}
trap cleanup INT TERM

# Wait for Gazebo (up to 60 s)
echo "[demo] Waiting for Gazebo..."
ELAPSED=0
until rosservice info /gazebo/set_model_state > /dev/null 2>&1; do
    sleep 2
    ELAPSED=$((ELAPSED + 2))
    if [ "$ELAPSED" -ge 60 ]; then
        echo "[demo] ERROR: Gazebo not ready after 60s — see /tmp/demo_gazebo.log"
        cleanup
    fi
done
echo "[demo] Gazebo ready (${ELAPSED}s)"

# Start fire oscillator in background
python3 src/com760cw2_group6/scripts/fire_oscillator.py &
OSC_PID=$!
echo "[demo] Fire oscillator started (PID $OSC_PID)"

echo ""
echo "  ================================================"
echo "  In a SECOND terminal run:"
echo "    python3 src/com760cw2_group6/scripts/bot0_demo.py"
echo "  Re-run that command as many times as you like."
echo "  Press Ctrl-C HERE to shut down everything."
echo "  ================================================"
echo ""

# Keep running until Ctrl-C (wait on the roslaunch process)
wait "$LAUNCH_PID"
cleanup

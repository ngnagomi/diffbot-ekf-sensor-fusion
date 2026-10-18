#!/bin/bash
# Record one test run: start recording, drive a pattern, stop recording.
# Usage:   ./tools/record_run.sh <run_name> [path_driver parameters...]

if [ -z "$1" ]; then
  echo "Usage: $0 <run_name> [-p name:=value ...]"
  exit 1
fi

NAME=$1
shift                              
RUNS_DIR=~/ekf_ws/runs
mkdir -p "$RUNS_DIR"

if [ -e "$RUNS_DIR/$NAME" ]; then
  echo "A run called '$NAME' already exists. Pick another name or delete it."
  exit 1
fi

# 1. Start recording in the background
ros2 bag record -o "$RUNS_DIR/$NAME" \
  /ground_truth/odom /wheel/odometry /odometry/filtered /imu/data /cmd_vel \
  /ekf_input/wheel/odometry /ekf_input/imu/data &
BAG_PID=$!
sleep 3                          

# 2. Drive the pattern
ros2 run diffbot_ekf path_driver --ros-args -p use_sim_time:=true "$@"

# 3. Record a moment of the robot at rest, then stop recording
sleep 2
kill -INT $BAG_PID
wait $BAG_PID
echo "Saved run to $RUNS_DIR/$NAME"
#!/bin/bash
# Run IMX219 publisher with automatic restart on failure

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.0.182:11311
export ROS_IP=192.168.0.182
export ROS_HOSTNAME=192.168.0.182

SCRIPT="$(dirname "$0")/imx219_publisher.py"

while true; do
    echo "[run_imx219] Restarting nvargus-daemon..."
    sudo systemctl restart nvargus-daemon
    sleep 3

    echo "[run_imx219] Starting publisher..."
    python3 "$SCRIPT"
    EXIT_CODE=$?

    echo "[run_imx219] Publisher exited with code $EXIT_CODE, restarting in 3s..."
    sleep 3
done

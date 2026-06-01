#!/bin/bash
# =============================================================================
#  record_calib_bag.sh — Countdown bag recorder for handheld calibration
#
#  Usage:
#    bash record_calib_bag.sh <bag_name> <duration_sec> [delay_sec] [topics...]
#
#  Examples:
#    # Camera intrinsics — 60s recording, 10s to get in position
#    bash record_calib_bag.sh ov9281_intrinsics 60 10 /camera/image_raw /camera/camera_info
#
#    # Camera-IMU extrinsics (Kalibr) — 120s recording, 10s delay
#    bash record_calib_bag.sh kalibr_ov9281_vxsimu 120 10 /camera/image_raw /camera/camera_info /imu
# =============================================================================

set -uo pipefail

BAG_NAME="${1:-calib}"
DURATION="${2:-60}"
DELAY="${3:-10}"
shift 3 2>/dev/null || true
TOPICS="${*:-/camera/image_raw /camera/camera_info}"

BAG_DIR="/home/vxs/vxs_ws/catkin_ws/src/vxs_sensor_ros1/record"
BAG_PATH="${BAG_DIR}/${BAG_NAME}.bag"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'

if [ -f "$BAG_PATH" ]; then
    echo -e "${YELLOW}[record_calib] WARNING: $BAG_PATH already exists — it will be overwritten.${NC}"
    sleep 2
fi

echo -e "${GREEN}[record_calib]${NC} Bag    : $BAG_PATH"
echo -e "${GREEN}[record_calib]${NC} Topics : $TOPICS"
echo -e "${GREEN}[record_calib]${NC} Record : ${DURATION}s"
echo ""
echo -e "${YELLOW}══════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}  GET READY — Pick up the sensor now!${NC}"
echo -e "${YELLOW}  Recording starts in ${DELAY}s${NC}"
echo -e "${YELLOW}══════════════════════════════════════════════════${NC}"

for i in $(seq "$DELAY" -1 1); do
    echo -ne "\r  ${YELLOW}Starting in ${i}s ...${NC}   "
    sleep 1
done
echo ""
echo -e "${GREEN}[record_calib] *** RECORDING STARTED — move the target! ***${NC}"

source /home/vxs/vxs_ws/catkin_ws/devel_container/setup.bash
export ROS_MASTER_URI=http://192.168.0.182:11311
export ROS_IP=192.168.0.182

rosbag record -O "$BAG_PATH" --duration="$DURATION" $TOPICS

echo ""
BAG_SIZE=$(du -sh "$BAG_PATH" 2>/dev/null | cut -f1 || echo "?")
echo -e "${GREEN}[record_calib] Done. Saved: $BAG_PATH ($BAG_SIZE)${NC}"

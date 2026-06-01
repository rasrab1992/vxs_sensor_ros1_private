#!/bin/bash
# =============================================================================
#  start_kalibr_bag_session.sh
#
#  Run INSIDE the container (user: vxs) OR on the host (user: etro).
#  Records OV9281 + VN-100 calibration bag entirely inside the container.
#
#  Usage:
#    bash start_kalibr_bag_session.sh [bag_name] [duration_sec]
#
#  Defaults:
#    bag_name     = cam_imu_calib_vn100
#    duration_sec = 120
# =============================================================================

set -o pipefail

DOCKER_NAME="agitated_mahavira"
ROS_MASTER_IP="192.168.0.182"
ROS_MASTER_PORT="11311"
VN100_PORT="/dev/ttyUSB0"
VN100_BAUD="921600"
OV9281_DEV="/dev/video1"

CONTAINER_WS="/home/vxs/vxs_ws/catkin_ws"
CONTAINER_DEVEL="${CONTAINER_WS}/devel_container/setup.bash"
BAG_DIR="${CONTAINER_WS}/src/vxs_sensor_ros1/record"

BAG_NAME="${1:-cam_imu_calib_vn100}"
DURATION="${2:-120}"

RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
log()  { echo -e "${GREEN}[kalibr-session]${NC} $*"; }
warn() { echo -e "${YELLOW}[kalibr-session]${NC} $*"; }
err()  { echo -e "${RED}[kalibr-session]${NC} $*" >&2; }

# ── Detect if we're inside or outside container ───────────────────────────────
if [ -f "/.dockerenv" ]; then
    # Running INSIDE the container — launch everything directly
    log "Running inside container — starting nodes directly"

    export ROS_MASTER_URI="http://${ROS_MASTER_IP}:${ROS_MASTER_PORT}"
    export ROS_IP="$ROS_MASTER_IP"
    export ROS_HOSTNAME="$ROS_MASTER_IP"
    source /opt/ros/noetic/setup.bash
    [ -f "$CONTAINER_DEVEL" ] && source "$CONTAINER_DEVEL"

    PIDS=()
    cleanup() {
        echo ""
        log "Shutting down..."
        for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
        rosnode kill /vectornav_driver_node /ov9281 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    # roscore
    if ! rostopic list &>/dev/null; then
        log "Starting roscore..."
        roscore &
        PIDS+=($!)
        sleep 3
    fi

    # OV9281
    log "Starting OV9281 ($OV9281_DEV)..."
    roslaunch vxs_sensor_ros1 ov9281.launch device:="$OV9281_DEV" &
    PIDS+=($!)

    log "Waiting for /camera/image_raw..."
    for i in $(seq 1 30); do
        rostopic info /camera/image_raw &>/dev/null && break
        sleep 1
        [ $i -eq 30 ] && { err "Timeout waiting for camera."; exit 1; }
    done
    log "  Camera live"

    # VN-100
    log "Starting VN-100 ($VN100_PORT)..."
    roslaunch vectornav_driver vectornav_driver_ros1.launch \
        port:="$VN100_PORT" baud_rate:="$VN100_BAUD" &
    PIDS+=($!)

    log "Waiting for /vectornav/imu/data..."
    for i in $(seq 1 30); do
        rostopic info /vectornav/imu/data &>/dev/null && break
        sleep 1
        [ $i -eq 30 ] && { err "Timeout waiting for IMU."; exit 1; }
    done
    log "  IMU live"

    # Countdown
    BAG_PATH="${BAG_DIR}/${BAG_NAME}.bag"
    echo ""
    echo -e "${YELLOW}════════════════════════════════════════════════════${NC}"
    echo -e "${YELLOW}  GET READY — ${DURATION}s recording starts in 10s${NC}"
    echo -e "${YELLOW}  Move sensor in figure-8, rotate all 3 axes${NC}"
    echo -e "${YELLOW}  Keep AprilGrid fully in view${NC}"
    echo -e "${YELLOW}════════════════════════════════════════════════════${NC}"
    for i in $(seq 10 -1 1); do echo -ne "\r  Starting in ${i}s ...   "; sleep 1; done
    echo ""

    log "Recording: $BAG_PATH"
    rosbag record -O "$BAG_PATH" --duration="$DURATION" \
        /camera/image_raw /camera/camera_info /vectornav/imu/data
    log "Done. $(du -sh "$BAG_PATH" | cut -f1)"

else
    # Running OUTSIDE the container — delegate everything into the container
    log "Running on host — delegating to container '$DOCKER_NAME'"

    if ! docker ps --format '{{.Names}}' | grep -q "^${DOCKER_NAME}$"; then
        err "Container '$DOCKER_NAME' is not running. Start it: docker start $DOCKER_NAME"
        exit 1
    fi

    SCRIPT_PATH="$(readlink -f "$0")"
    # Copy this script into container and run it inside
    docker cp "$SCRIPT_PATH" "${DOCKER_NAME}:/tmp/start_kalibr_bag_session.sh"
    docker exec -it "$DOCKER_NAME" bash /tmp/start_kalibr_bag_session.sh "$BAG_NAME" "$DURATION"
fi

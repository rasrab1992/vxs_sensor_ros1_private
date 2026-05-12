#!/bin/bash
# =============================================================================
#  run_kalibr_calibration.sh
#
#  Full Kalibr camera-IMU calibration pipeline:
#    1. Record calibration bag (camera + IMU) — skipped if bag already exists
#    2. Run kalibr_calibrate_imu_camera inside kalibr_arm64 Docker container
#    3. Print result location
#
#  Usage:
#    bash run_kalibr_calibration.sh [bag_name] [duration_sec]
#
#  Defaults:
#    bag_name     = kalibr_calib  (saved to record/)
#    duration_sec = 180           (3 minutes)
#
#  To skip recording and use an existing bag:
#    bash run_kalibr_calibration.sh kalibr_calib skip
# =============================================================================

set -uo pipefail

# ── Config ───────────────────────────────────────────────────────────────────
DOCKER_NAME="agitated_mahavira"
KALIBR_IMAGE="kalibr_arm64"
ROS_MASTER_IP="192.168.0.182"
ROS_MASTER_PORT="11311"
VN100_PORT="/dev/ttyUSB0"
VN100_BAUD="921600"

HOST_WS="/home/etro/vxs_ws/catkin_ws"
BAG_DIR="${HOST_WS}/src/vxs_sensor_ros1/record"
CALIB_DIR="${HOST_WS}/src/vxs_sensor_ros1/calibration"
IMX219_PUBLISHER="${HOST_WS}/src/vxs_sensor_ros1/scripts/imx219_publisher.py"
HOST_DEVEL="${HOST_WS}/devel_host/setup.bash"
CONTAINER_DEVEL="/home/vxs/vxs_ws/catkin_ws/devel_container/setup.bash"

BAG_NAME="${1:-kalibr_calib}"
DURATION="${2:-180}"

BAG_PATH="${BAG_DIR}/${BAG_NAME}.bag"
RESULT_DIR="${BAG_DIR}/kalibr_results"

# Kalibr config files (mounted into container)
CAMCHAIN_YAML="${CALIB_DIR}/kalibr_camera_chain.yaml"
IMU_YAML="${CALIB_DIR}/kalibr_imu.yaml"
TARGET_YAML="${CALIB_DIR}/aprilgrid_6x6.yaml"

# ── Colours ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; CYAN='\033[0;36m'; NC='\033[0m'
log()  { echo -e "${GREEN}[kalibr]${NC} $*"; }
warn() { echo -e "${YELLOW}[kalibr]${NC} $*"; }
err()  { echo -e "${RED}[kalibr]${NC} $*" >&2; }
hdr()  { echo -e "\n${CYAN}══════════════════════════════════════════════════${NC}"; \
         echo -e "${CYAN}  $*${NC}"; \
         echo -e "${CYAN}══════════════════════════════════════════════════${NC}"; }

PIDS=()
cleanup() {
    echo ""
    log "Shutting down recording processes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    docker exec "$DOCKER_NAME" bash -c \
        "source /opt/ros/noetic/setup.bash && \
         export ROS_MASTER_URI=http://${ROS_MASTER_IP}:${ROS_MASTER_PORT} && \
         rosnode kill /vectornav_driver_node 2>/dev/null || true" \
        2>/dev/null || true
}

# ── Sanity checks ────────────────────────────────────────────────────────────
for f in "$CAMCHAIN_YAML" "$IMU_YAML" "$TARGET_YAML"; do
    [ -f "$f" ] || { err "Missing config file: $f"; exit 1; }
done

if ! docker image inspect "$KALIBR_IMAGE" &>/dev/null; then
    err "Docker image '$KALIBR_IMAGE' not found. Build it first with Dockerfile_arm64."
    exit 1
fi

mkdir -p "$RESULT_DIR"

# =============================================================================
# PHASE 1 — Record bag (skip if bag exists or DURATION=skip)
# =============================================================================
if [ "$DURATION" = "skip" ] || [ -f "$BAG_PATH" ]; then
    if [ -f "$BAG_PATH" ]; then
        warn "Bag already exists: $BAG_PATH — skipping recording."
        warn "Delete it to re-record, or pass a different bag name."
    else
        err "DURATION=skip but bag not found: $BAG_PATH"
        exit 1
    fi
else
    hdr "PHASE 1 — Recording calibration bag (${DURATION}s)"

    # Prerequisites
    if ! command -v docker &>/dev/null; then { err "docker not found"; exit 1; }; fi
    if ! docker ps --format '{{.Names}}' | grep -q "^${DOCKER_NAME}$"; then
        err "Container '${DOCKER_NAME}' is not running. Start it first: docker start ${DOCKER_NAME}"
        exit 1
    fi

    export ROS_MASTER_URI="http://${ROS_MASTER_IP}:${ROS_MASTER_PORT}"
    export ROS_IP="$ROS_MASTER_IP"
    export ROS_HOSTNAME="$ROS_MASTER_IP"
    source /opt/ros/noetic/setup.bash
    [ -f "$HOST_DEVEL" ] && source "$HOST_DEVEL"

    # roscore
    if ! rostopic list &>/dev/null; then
        log "Starting roscore..."
        roscore &
        PIDS+=($!)
        sleep 3
    fi

    # Camera
    log "Restarting nvargus-daemon and starting IMX219 publisher..."
    pkill -f imx219_publisher.py 2>/dev/null || true
    sleep 1
    sudo systemctl restart nvargus-daemon
    sleep 6
    python3 "$IMX219_PUBLISHER" &
    PIDS+=($!)

    log "Waiting for /camera/image_raw publisher (up to 60s)..."
    for i in $(seq 1 60); do
        PUB_COUNT=$(rostopic info /camera/image_raw 2>/dev/null | grep -c "imx219_publisher" || true)
        if [ "$PUB_COUNT" -gt 0 ]; then
            break
        fi
        sleep 1
        [ $i -eq 60 ] && { err "Timed out waiting for camera publisher."; exit 1; }
    done
    log "  Camera publisher up — waiting 5s for first frames..."
    sleep 5
    log "  Camera live"

    # IMU
    log "Starting VN-100 inside ${DOCKER_NAME}..."
    if ! docker ps --format '{{.Names}}' | grep -q "^${DOCKER_NAME}$"; then
        err "Container '${DOCKER_NAME}' is not running. Start it: docker start ${DOCKER_NAME}"
        exit 1
    fi
    # Kill any stale vectornav process in container first
    docker exec "$DOCKER_NAME" bash -c "pkill -f vectornav_driver_ros1 2>/dev/null || true" 2>/dev/null || true
    sleep 2
    docker exec -d "$DOCKER_NAME" bash -c "
        source /opt/ros/noetic/setup.bash
        [ -f ${CONTAINER_DEVEL} ] && source ${CONTAINER_DEVEL} || true
        export ROS_MASTER_URI=http://${ROS_MASTER_IP}:${ROS_MASTER_PORT}
        export ROS_IP=${ROS_MASTER_IP}
        export ROS_HOSTNAME=${ROS_MASTER_IP}
        roslaunch vectornav_driver vectornav_driver_ros1.launch \
            port:=${VN100_PORT} baud_rate:=${VN100_BAUD} \
            > /tmp/vectornav_launch.log 2>&1
    "
    log "  Waiting for /vectornav/imu/data (up to 30s)..."
    for i in $(seq 1 30); do
        if rostopic info /vectornav/imu/data &>/dev/null; then
            break
        fi
        sleep 1
        if [ $i -eq 15 ]; then
            warn "  Still waiting... checking container log:"
            docker exec "$DOCKER_NAME" tail -5 /tmp/vectornav_launch.log 2>/dev/null || true
        fi
        if [ $i -eq 30 ]; then
            err "Timed out waiting for IMU. Container log:"
            docker exec "$DOCKER_NAME" tail -20 /tmp/vectornav_launch.log 2>/dev/null || true
            exit 1
        fi
    done
    log "  IMU live"

    # Rates
    echo ""
    log "Verifying topic rates before recording..."
    CAM_HZ=$(timeout 5 rostopic hz /camera/image_raw 2>/dev/null | grep -oP 'average rate: \K[0-9.]+' | head -1)
    IMU_HZ=$(timeout 5 rostopic hz /vectornav/imu/data 2>/dev/null | grep -oP 'average rate: \K[0-9.]+' | head -1)
    log "  Camera: ${CAM_HZ:-?} Hz  (target ~10)"
    log "  IMU:    ${IMU_HZ:-?} Hz  (target ~200)"

    echo ""
    echo -e "${YELLOW}════════════════════════════════════════════════════${NC}"
    echo -e "${YELLOW}  STARTING RECORDING IN 5s — ${DURATION}s total${NC}"
    echo -e "${YELLOW}  Keep checkerboard visible, move slowly.${NC}"
    echo -e "${YELLOW}  Figure-8, rotate all 3 axes.${NC}"
    echo -e "${YELLOW}════════════════════════════════════════════════════${NC}"
    for i in 10 9 8 7 6 5 4 3 2 1; do
        echo -e "${YELLOW}  Starting in ${i}...${NC}"
        sleep 1
    done

    trap cleanup EXIT INT TERM

    log "Recording to ${BAG_PATH} ..."
    rosbag record \
        -O "$BAG_PATH" \
        --duration="${DURATION}" \
        /camera/image_raw \
        /camera/camera_info \
        /vectornav/imu/data
    log "Recording done."

    # Stop recording processes
    cleanup
    trap - EXIT INT TERM
    PIDS=()
fi

# =============================================================================
# PHASE 2 — Run Kalibr inside Docker
# =============================================================================
hdr "PHASE 2 — Running Kalibr calibration"

BAG_SIZE=$(du -sh "$BAG_PATH" | cut -f1)
log "Bag: $BAG_PATH ($BAG_SIZE)"
log "Results will be saved to: $RESULT_DIR"

echo ""
log "This typically takes 5-20 minutes depending on bag length..."
echo ""

docker run --rm \
    --entrypoint bash \
    -v "${BAG_DIR}:/bags" \
    -v "${CALIB_DIR}:/calib" \
    -v "${RESULT_DIR}:/results" \
    "$KALIBR_IMAGE" \
    -c "
        set -e
        source /opt/ros/noetic/setup.bash
        source /kalibr_ws/devel/setup.bash
        echo '[kalibr] Starting kalibr_calibrate_imu_camera...'
        cd /results
        rosrun kalibr kalibr_calibrate_imu_camera \
            --bag /bags/${BAG_NAME}.bag \
            --cams /calib/kalibr_camera_chain.yaml \
            --imu /calib/kalibr_imu.yaml \
            --target /calib/kalibr_target.yaml \
            --dont-show-report
        echo '[kalibr] Done.'
    "

# =============================================================================
# PHASE 3 — Print results
# =============================================================================
hdr "PHASE 3 — Results"

if ls "$RESULT_DIR"/*.yaml 2>/dev/null | head -1 | grep -q yaml; then
    log "Calibration complete. Output files:"
    ls -lh "$RESULT_DIR"/
    echo ""
    RESULT_YAML=$(ls "$RESULT_DIR"/*camchain-imucam*.yaml 2>/dev/null | head -1)
    if [ -n "$RESULT_YAML" ]; then
        echo -e "${GREEN}── Camera-IMU transform (T_cam_imu) ──${NC}"
        grep -A 20 "T_cam_imu" "$RESULT_YAML" | head -25
    fi
else
    warn "No YAML results found in $RESULT_DIR — check Kalibr output above for errors."
fi

log "All done."

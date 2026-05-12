#!/bin/bash
# =============================================================================
#  start_kalibr_bag_session.sh
#
#  Run on the HOST (Jetson, user: etro).  Starts all components needed to
#  record a Kalibr camera-IMU calibration bag:
#
#    1. roscore          (host — skipped if already running)
#    2. IMX219 camera    (host — imx219_publisher.py via nvargus-daemon)
#    3. VN-100 IMU       (docker container agitated_mahavira)
#    4. rosbag record    (host) — /camera/image_raw + /vectornav/imu/data
#
#  Layout:
#    host      : /home/etro/vxs_ws/catkin_ws/src/vxs_sensor_ros1/
#    container : /home/vxs/vxs_ws/catkin_ws/src/vxs_sensor_ros1/
#
#  Usage:
#    bash start_kalibr_bag_session.sh [bag_name] [duration_sec]
#
#  Defaults:
#    bag_name     = cam_imu_calib_vn100
#    duration_sec = 120   (0 = unlimited, stop with Ctrl-C)
# =============================================================================

set -uo pipefail

# ── Configurable ─────────────────────────────────────────────────────────────
DOCKER_NAME="agitated_mahavira"
ROS_MASTER_IP="192.168.0.182"
ROS_MASTER_PORT="11311"
VN100_PORT="/dev/ttyUSB0"
VN100_BAUD="921600"

# HOST paths  (user: etro)
HOST_WS="/home/etro/vxs_ws/catkin_ws"
BAG_DIR="${HOST_WS}/src/vxs_sensor_ros1/record"
IMX219_PUBLISHER="${HOST_WS}/src/vxs_sensor_ros1/scripts/imx219_publisher.py"
HOST_DEVEL="${HOST_WS}/devel_host/setup.bash"

# CONTAINER paths  (user: vxs)
CONTAINER_DEVEL="/home/vxs/vxs_ws/catkin_ws/devel_container/setup.bash"

BAG_NAME="${1:-cam_imu_calib_vn100}"
DURATION="${2:-120}"

# ── Colours ──────────────────────────────────────────────────────────────────
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
log()  { echo -e "${GREEN}[kalibr-session]${NC} $*"; }
warn() { echo -e "${YELLOW}[kalibr-session]${NC} $*"; }
err()  { echo -e "${RED}[kalibr-session]${NC} $*" >&2; }

# ── Tracked PIDs ─────────────────────────────────────────────────────────────
PIDS=()

cleanup() {
    echo ""
    log "Shutting down all processes..."
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null && log "  killed PID $pid" || true
    done
    docker exec "$DOCKER_NAME" bash -c \
        "source /opt/ros/noetic/setup.bash && \
         export ROS_MASTER_URI=http://${ROS_MASTER_IP}:${ROS_MASTER_PORT} && \
         rosnode kill /vectornav_driver_node 2>/dev/null || true" \
        2>/dev/null || true
    log "Done."
}
trap cleanup EXIT INT TERM

# ── Sanity checks ────────────────────────────────────────────────────────────
log "Checking prerequisites..."

if ! command -v docker &>/dev/null; then
    err "docker not found in PATH."
    exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -q "^${DOCKER_NAME}$"; then
    err "Container '${DOCKER_NAME}' is not running."
    err "Start it with:  docker start ${DOCKER_NAME}"
    exit 1
fi

if [ ! -c "$VN100_PORT" ]; then
    warn "VN-100 port $VN100_PORT not found — driver will fail if not connected."
fi

# ── ROS env (host) ───────────────────────────────────────────────────────────
export ROS_MASTER_URI="http://${ROS_MASTER_IP}:${ROS_MASTER_PORT}"
export ROS_IP="$ROS_MASTER_IP"
export ROS_HOSTNAME="$ROS_MASTER_IP"

source /opt/ros/noetic/setup.bash
[ -f "$HOST_DEVEL" ] && source "$HOST_DEVEL"

# =============================================================================
# 1. roscore — start only if not already running
# =============================================================================
log "Step 1/4 — Checking roscore..."
if rostopic list &>/dev/null; then
    log "  Already running at $ROS_MASTER_URI — skipping"
else
    log "  Starting roscore..."
    roscore &
    PIDS+=($!)
    sleep 3
    log "  roscore PID ${PIDS[-1]}"
fi

# =============================================================================
# 2. IMX219 camera (host)
# =============================================================================
log "Step 2/4 — Killing stale camera processes, restarting nvargus-daemon, starting IMX219 publisher..."
pkill -f gscam 2>/dev/null || true
pkill -f imx219_publisher.py 2>/dev/null || true
sleep 1  # let processes die before daemon restart
sudo systemctl restart nvargus-daemon
sleep 6  # Argus socket needs time to fully initialise

python3 "$IMX219_PUBLISHER" &
PIDS+=($!)
log "  imx219_publisher.py PID ${PIDS[-1]}"

log "  Waiting for first camera frame on /camera/image_raw..."
TIMEOUT=40; ELAPSED=0
while ! timeout 2 rostopic echo --count=1 /camera/image_raw &>/dev/null; do
    sleep 1; ELAPSED=$((ELAPSED+1))
    if [ $ELAPSED -ge $TIMEOUT ]; then
        err "Timed out waiting for /camera/image_raw frames — check nvargus-daemon."
        exit 1
    fi
done
log "  /camera/image_raw is live (${ELAPSED}s)"

# =============================================================================
# 3. VN-100 IMU — inside Docker container
# =============================================================================
log "Step 3/4 — Starting VN-100 driver inside ${DOCKER_NAME}..."
docker exec -d "$DOCKER_NAME" bash -c "
    source /opt/ros/noetic/setup.bash
    [ -f ${CONTAINER_DEVEL} ] && source ${CONTAINER_DEVEL} || true
    export ROS_MASTER_URI=http://${ROS_MASTER_IP}:${ROS_MASTER_PORT}
    export ROS_IP=${ROS_MASTER_IP}
    export ROS_HOSTNAME=${ROS_MASTER_IP}
    roslaunch vectornav_driver vectornav_driver_ros1.launch \
        port:=${VN100_PORT} \
        baud_rate:=${VN100_BAUD}
"

log "  Waiting for /vectornav/imu/data..."
TIMEOUT=20; ELAPSED=0
while ! rostopic info /vectornav/imu/data &>/dev/null; do
    sleep 1; ELAPSED=$((ELAPSED+1))
    if [ $ELAPSED -ge $TIMEOUT ]; then
        err "Timed out waiting for /vectornav/imu/data."
        err "  Debug: docker exec ${DOCKER_NAME} rosnode list"
        exit 1
    fi
done
log "  /vectornav/imu/data is live (${ELAPSED}s)"

# =============================================================================
# 4. rosbag record (host)
# =============================================================================
BAG_PATH="${BAG_DIR}/${BAG_NAME}.bag"
log "Step 4/4 — Recording bag: ${BAG_PATH}"

DURATION_ARG=""
if [ "${DURATION}" -gt 0 ] 2>/dev/null; then
    DURATION_ARG="--duration=${DURATION}"
    log "  Duration: ${DURATION}s"
else
    log "  Duration: unlimited (Ctrl-C to stop)"
fi

rosbag record \
    -O "$BAG_PATH" \
    $DURATION_ARG \
    /camera/image_raw \
    /camera/camera_info \
    /vectornav/imu/data &
PIDS+=($!)
log "  rosbag PID ${PIDS[-1]}"

# =============================================================================
# Status
# =============================================================================
echo ""
echo -e "${GREEN}════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  Kalibr bag session running${NC}"
echo -e "${GREEN}════════════════════════════════════════════════════════${NC}"
echo "  Camera  : /camera/image_raw    (host — imx219_publisher.py)"
echo "  IMU     : /vectornav/imu/data  (docker — vectornav_driver)"
echo "  Bag     : ${BAG_PATH}"
[ "${DURATION}" -gt 0 ] 2>/dev/null && \
    echo "  Stops in: ${DURATION}s" || \
    echo "  Stop    : Ctrl-C"
echo ""
echo "  Check rates:"
echo "    rostopic hz /camera/image_raw"
echo "    rostopic hz /vectornav/imu/data"
echo ""

wait "${PIDS[-1]}" 2>/dev/null || true
log "Recording finished."

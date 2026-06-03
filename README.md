# ROS1 Package for the VXS Sensor

A ROS1 package for publishing sensor data from the VoxelSensors Andromeda2 sensor
(depth image, point cloud, event stream, IMU).

---

## Hardware Setup

| Component | Details |
|-----------|---------|
| Compute | Jetson Orin NX 16G (Seeed J401) — migrating to Orin Nano Super |
| Sensor | VXS Andromeda2 (Lissajous LiDAR, USB) |
| IMU (external) | VectorNav VN-100 (USB/FTDI, 200 Hz) |
| Camera | IMX219 CSI (Jetson CAM1) or OV2311 USB (recommended) |
| ROS master | `http://192.168.0.182:11311` (Jetson) |
| Visualization | RViz on WSL: `192.168.0.164` |

---

## Docker Setup

### Install Docker + NVIDIA Container Toolkit

```bash
# Install Docker
sudo apt-get install -y docker.io
sudo groupadd docker
sudo usermod -aG docker $USER
sudo systemctl restart docker

# Install NVIDIA Container Toolkit
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt update && sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### Pull Pre-built VXS Image

```bash
# AMD64 with NVIDIA GPU
docker pull terzakig/vxsros1:amd64

# AMD64 without GPU
docker pull terzakig/vxsros1:amd64_no_gpu

# ARM64 — Jetson (Orin NX / Orin Nano Super)
docker pull terzakig/vxsros1:arm64
```

### Run the Container

```bash
# ARM64 (Jetson) — use the provided script
bash docker/run_arm64.sh terzakig/vxsros1:arm64

# AMD64 with GPU
bash docker/run.sh terzakig/vxsros1:amd64
```

The script mounts:
- `~/vxs_ws` → workspace (shared with host)
- `~/sandbox` → scratch space
- `/dev` → USB/device access

> Create `~/vxs_ws/catkin_ws` and `~/sandbox` on the host before running if they do not exist.

### Export Container as Image (for migration to new Jetson)

```bash
# Commit the current running container state
docker commit agitated_mahavira vxs_ros1_noetic:backup

# Save to a compressed tarball
docker save vxs_ros1_noetic:backup | gzip > ~/vxs_ros1_noetic_backup.tar.gz

# Transfer to new Jetson (SCP or USB drive)
scp ~/vxs_ros1_noetic_backup.tar.gz <user>@<new_jetson_ip>:~/

# Load on new Jetson
docker load < ~/vxs_ros1_noetic_backup.tar.gz
docker images   # verify image appears

# Run on new Jetson
bash docker/run_arm64.sh vxs_ros1_noetic:backup
```

---

## Workspace Setup (inside container)

```bash
# Create workspace directories on host first (shared via volume mount)
mkdir -p ~/vxs_ws/catkin_ws/src ~/sandbox

# Enter the container
bash docker/run_arm64.sh terzakig/vxsros1:arm64

# Inside container — clone the package
cd ~/vxs_ws/catkin_ws/src
git clone https://github.com/rasrab1992/vxs_sensor_ros1_private.git vxs_sensor_ros1

# Populate workspace with rosinstall (ARM64)
cp vxs_sensor_ros1/rosinstall/rosinstall_arm64 ./.rosinstall
rosinstall .

# Build
cd ~/vxs_ws/catkin_ws
catkin build
```

> **Separate build spaces for host vs container** (avoids path conflicts):
> - Host (`etro` user): `catkin config --build-space build_host --devel-space devel_host`
> - Container (`vxs` user): `catkin config --build-space build_container --devel-space devel_container`

---

## VN-100 IMU Driver (inside container)

The `ros-noetic-vectornav` apt package does not exist for arm64. Build from source:

```bash
# One-time setup
sudo apt-get install -y libspdlog-dev
rm ~/vxs_ws/catkin_ws/src/vectornav/CATKIN_IGNORE

source /opt/ros/noetic/setup.bash
cd ~/vxs_ws/catkin_ws
catkin config --build-space build_container --devel-space devel_container \
  --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
catkin build vectornav_driver

# Run the driver
source devel_container/setup.bash
export ROS_MASTER_URI=http://192.168.0.182:11311
export ROS_IP=192.168.0.182
roslaunch vectornav_driver vectornav_driver_ros1.launch port:=/dev/ttyUSB0 baud_rate:=921600
# Publishes /vectornav/imu/data at 200 Hz
```

Install udev rule so the device always appears at `/dev/vn100`:
```bash
sudo cp ~/vxs_ws/catkin_ws/src/vectornav/vectornav_driver/udev/99-vn100.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## IMX219 Camera Publisher (Jetson host, `etro` user)

The IMX219 CSI camera runs on the **host** (not in Docker) because `nvargus-daemon`
manages the Jetson ISP hardware.

```bash
# Allow publisher to restart nvargus-daemon without password (run once)
echo "etro ALL=(ALL) NOPASSWD: /bin/systemctl restart nvargus-daemon" | \
  sudo tee /etc/sudoers.d/nvargus-restart

# Start the publisher
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.0.182:11311
export ROS_IP=192.168.0.182
python3 /home/etro/vxs_ws/catkin_ws/src/vxs_sensor_ros1/scripts/imx219_publisher.py
# Publishes /camera/image_raw (1280×720 BGR @ 10 Hz) + /camera/camera_info
```

The publisher uses `sensor-mode=2` (1920×1080 captured, scaled to 1280×720).
It auto-restarts `nvargus-daemon` and re-execs itself on Argus TIMEOUT.

> **Orin NX only:** Requires DTB patch to disable ghost `module1`.
> See `calibration/CALIBRATION_PIPELINE.md` Step 1 for details.
> The Orin Nano Super (JP6.x) does not require this patch.

---

## Running the VXS Node

```bash
# USB access — one-time setup (required or LibUSB reports "no access")
sudo usermod -aG plugdev $USER
# Log out and back in, then verify: groups | grep plugdev

# Event streaming mode (recommended)
roslaunch vxs_sensor_ros1 vxs_events.launch publish_imu:=false

# Event streaming + IMU
roslaunch vxs_sensor_ros1 vxs_events.launch publish_imu:=true

# With VN-100 external IMU
roslaunch vxs_sensor_ros1 vxs_vn100_li_init_online.launch
```

### Node Arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `publish_depth_image` | bool | false | Publish depth image (frame mode only) |
| `publish_pointcloud` | bool | false | Publish XYZ point cloud (frame mode only) |
| `publish_events` | bool | false | Publish XYZT event cloud (streaming mode) — overrides depth/pcloud |
| `publish_imu` | bool | false | Publish IMU samples (streaming mode only) |
| `fps` | int | 33 | Frame rate (streaming: event window period = 1000/fps ms) |
| `config_json` | string | — | Full path to SDK config JSON |
| `calib_json` | string | — | Full path to calibration JSON |
| `binning_amount` | int | 0 | Spatial binning (0=full resolution, higher=coarser) |
| `prefiltering_threshold` | float | 2.0 | Pre-filter threshold |
| `filterP1` | float | 0.1 | Filter parameter P1 |
| `temporal_threshold` | int | 4 | Temporal filter threshold |
| `spatial_threshold` | int | 10 | Spatial filter threshold |
| `on_time` | int | 0 | Observation window on-time in SDK ticks (0 = continuous) |
| `period_time` | int | 0 | Observation window period in SDK ticks (0 = continuous) |

### Observation Window (Point Cloud Density)

The observation window controls how long the sensor actively integrates light per cycle.
A longer `on_time` relative to `period_time` gives denser point clouds.

**Recommended settings for GOLDEN mode:**

| Use case | on_time | period_time | Duty cycle |
|----------|---------|-------------|------------|
| Continuous (default) | 0 | 0 | 100% |
| Best density + quality | 100 | 200 | 50% |
| Low power | 30 | 60 | 50% |

> **SDK note (2026-06-03):** `vxSetObservationWindow` **must be called after `vxStartSystem`**,
> not before. In the new SDK version (updated May 2026), calling it before `vxStartSystem`
> is silently ignored, resulting in no effect on point cloud density. This is fixed in
> `src/publisher/vxs_node.cpp`. The old SDK accepted both orderings.

> **Point cloud units:** The VXS SDK returns coordinates in **millimeters**. The ROS node
> converts to **meters** (`* 1e-3`) before publishing on `/vxs/pcloud/events`.

> **IMU note:** The VXS built-in IIM-42652 IMU has a confirmed firmware bug —
> `vxIMU.timestamp` is always zero on SDK 2.8 (`5-updated_sdk` branch).
> Use the external VN-100 IMU for timestamped IMU data until the firmware is fixed.

### Topics Published

| Topic | Type | Mode |
|-------|------|------|
| `/vxs/pcloud/events` | sensor_msgs/PointCloud2 | Events (XYZT) |
| `/vxs/sensor/camera_info` | sensor_msgs/CameraInfo | Always |
| `/depth/image` | sensor_msgs/Image | Frame mode |
| `/pcloud/cloud` | sensor_msgs/PointCloud2 | Frame mode |
| `/imu` | sensor_msgs/Imu | publish_imu:=true |

---

## Calibration

Full pipeline documented in `calibration/CALIBRATION_PIPELINE.md`.

### Quick Summary

1. **IMX219 intrinsics** — ✅ Done (`calibration/imx219_intrinsics.yaml`)
2. **VN-100 Allan variance** — ✅ Done (`calibration/vn100_kalibr.yaml`, 49-min bag)
3. **Camera-IMU (Kalibr)** — ⏳ Record bag with `scripts/start_kalibr_bag_session.sh`
4. **VXS-Camera extrinsics** — ⏳ AprilGrid target-based
5. **Transform chaining** — ⏳ `T_VXS_I = T_VXS_C @ T_C_I`

```bash
# One-command calibration bag recording (host, etro user)
bash /home/etro/vxs_ws/catkin_ws/src/vxs_sensor_ros1/scripts/start_kalibr_bag_session.sh
# Records /camera/image_raw + /vectornav/imu/data for 120s
# Output: record/cam_imu_calib_vn100.bag
```

---

## ROS Network Configuration

### Jetson Container (`vxs` user)
```bash
export ROS_MASTER_URI=http://192.168.0.182:11311
export ROS_IP=192.168.0.182
export ROS_HOSTNAME=192.168.0.182
```

### Jetson Host (`etro` user)
```bash
export ROS_MASTER_URI=http://192.168.0.182:11311
export ROS_IP=192.168.0.182
export ROS_HOSTNAME=192.168.0.182
```

### WSL Laptop (`192.168.0.164`)
```bash
# ~/.bashrc
export ROS_MASTER_URI=http://192.168.0.182:11311
export ROS_IP=192.168.0.164
export ROS_HOSTNAME=192.168.0.164
# Requires WSL mirrored networking in C:\Users\<user>\.wslconfig:
#   [wsl2]
#   networkingMode=mirrored
```

---

## Observe Topics

```bash
rostopic list
rostopic hz /vxs/pcloud/events
rostopic hz /vectornav/imu/data
rostopic hz /camera/image_raw
rqt_image_view   # view camera image
```

---

## Key Scripts

| Script | Where to run | Description |
|--------|-------------|-------------|
| `docker/run_arm64.sh` | Host | Start VXS Docker container (ARM64/Jetson) |
| `scripts/imx219_publisher.py` | Host (`etro`) | IMX219 CSI camera → ROS topics |
| `scripts/start_kalibr_bag_session.sh` | Host (`etro`) | Record Kalibr calibration bag |
| `scripts/run_imx219.sh` | Host (`etro`) | Wrapper for imx219_publisher.py with auto-restart |

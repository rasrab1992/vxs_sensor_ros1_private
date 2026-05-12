# VXS Sensor Calibration Pipeline
## Platform: Jetson Orin NX 16G, Seeed J401, ROS1 Noetic (Docker)
## Updated: 2026-05-12

---

## Hardware

| Component | Details |
|-----------|---------|
| Compute | Jetson Orin NX 16G on Seeed J401 carrier board |
| LiDAR | VXS Andromeda2 (Lissajous scan pattern) |
| IMU (Kalibr) | VectorNav VN-100 (external, USB, published on `/imu` via vectornav driver) |
| IMU (VXS) | VXS built-in IIM-42652 — **on hold** (SDK firmware bug: `timestamp` field always zero) |
| Camera (CSI) | Raspberry Pi Camera V2 (IMX219 8MP, CSI) — Argus unstable on JP5.1.3 |
| Camera (USB, recommended) | Arducam OV2311 Global Shutter USB (1600×1200, monochrome) |
| Camera (legacy ref) | Trust USB Webcam 640×480 YUYV |
| ROS master | Container on Jetson: `http://192.168.0.182:11311` |
| Visualization | RViz on WSL laptop: `192.168.0.164` |

---

## Transform Chain Goal

```
T_C_I   = camera ← VN-100 IMU         (from Kalibr)
T_VXS_C = VXS point cloud ← camera    (from target-based calibration)
T_VXS_I = T_VXS_C * T_C_I             (chained)
```

> **Note on VXS built-in IMU:** The IIM-42652 IMU inside the VXS sensor has a confirmed
> firmware bug: the `vxIMU.timestamp` field is always zero on this SDK version (2.8,
> branch `5-updated_sdk`). This makes hardware-timestamped IMU data impossible. The company
> has been notified. Once the firmware is fixed, the pipeline can switch to using the
> VXS IMU directly (removing the need for the external VN-100 and the T_VN100_VXS transform).

---

## Step 1 — IMX219 Camera Driver Installation

### 1.1 Physical Connection (Seeed J401)
1. Pull out the black ZIF lock on the CSI connector (CAM1 port)
2. Insert the 15-pin ribbon cable with **gold fingers facing downward**
3. Push the black lock back to secure

### 1.2 Configure Device Tree (two-step: enable + patch)

**Step A — enable the overlay:**
```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
# Select "Camera IMX219 Dual" → save (do NOT reboot yet)
```

**Step B — patch DTB to disable the ghost module1** (prevents 5.5s Argus TIMEOUT):
```bash
sudo dtc -I dtb -O dts \
  /boot/kernel_tegra234-p3767-0000-p3509-a02-user-custom.dtb \
  -o /tmp/custom.dts 2>/dev/null

python3 - <<'EOF'
import re
dts = open('/tmp/custom.dts').read()
def disable_module1(text):
    result, depth, in_tcp, in_modules, in_m1, m1_depth = [], 0, False, False, False, 0
    for line in text.splitlines(keepends=True):
        s = line.strip()
        if 'tegra-camera-platform' in s: in_tcp = True
        if in_tcp and 'modules {' in s: in_modules = True
        if in_modules and re.match(r'\s*module1\s*\{', line): in_m1, m1_depth = True, depth
        if in_m1 and 'status' in s:
            line = re.sub(r'status\s*=\s*"[^"]*"', 'status = "disabled"', line)
        depth += line.count('{') - line.count('}')
        if in_m1 and depth <= m1_depth and '}' in s: in_m1 = False
        result.append(line)
    return ''.join(result)
open('/tmp/patched.dts', 'w').write(disable_module1(dts))
print("Done")
EOF

sudo dtc -I dts -O dtb /tmp/patched.dts \
  -o /boot/kernel_tegra234-p3767-0000-p3509-a02-user-custom.dtb 2>/dev/null
sudo reboot
```

**Why:** "Camera IMX219 Dual" tells Argus to wait for TWO sensors. CAM1 only has one.
The ghost second sensor causes Argus to TIMEOUT after ~330 frames (5.5s @ 60fps).
Patching `module1 { status = "disabled" }` removes the ghost.

**Verify after reboot:**
```bash
cat /proc/device-tree/tegra-camera-platform/modules/module1/status
# Must print: disabled
v4l2-ctl --list-devices   # should show /dev/video0
```

### 1.3 Allow publisher to restart nvargus-daemon without password (run once)
```bash
echo "etro ALL=(ALL) NOPASSWD: /bin/systemctl restart nvargus-daemon" | \
  sudo tee /etc/sudoers.d/nvargus-restart
```

### 1.4 Test GStreamer Pipeline
```bash
# sensor-mode=2 (1920×1080@30fps) — stable on JP5.1.3
gst-launch-1.0 nvarguscamerasrc sensor-id=0 sensor-mode=2 num-buffers=30 \
  ! 'video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1' \
  ! nvvidconv ! 'video/x-raw(memory:NVMM),width=1280,height=720,format=NV12' \
  ! nvvidconv ! 'video/x-raw,format=I420' \
  ! fakesink sync=false
# Should complete without TIMEOUT or ISP IVC errors
```

> **ISP IVC crash warning**: If you see `tegra194-isp5: failed to register control callback`
> in dmesg and nvargus-daemon SEGVs, only a full reboot clears it. Do NOT attempt to
> disable `rbpcv2_imx219_c@10` in the DTB — this breaks the RTCPU HSP IVC channel
> and makes Argus permanently non-functional. The module1 patch above is sufficient.

### 1.5 Run IMX219 ROS Publisher (Jetson host, etro user)
```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.0.182:11311
export ROS_IP=192.168.0.182
export ROS_HOSTNAME=192.168.0.182
python3 /home/etro/vxs_ws/catkin_ws/src/vxs_sensor_ros1/scripts/imx219_publisher.py
```

Script: `scripts/imx219_publisher.py`
- Pipeline: `nvarguscamerasrc sensor-mode=2 1920×1080 → scale 1280×720 in NVMM → I420 → videorate 10fps → fakesink signal-handoffs=true`
- Python receives I420 frames via `fakesink handoff` signal (no NVMM DMA-fd issues)
- Publishes `/camera/image_raw` (bgr8), `/camera/image_raw/compressed` (jpeg), `/camera/camera_info` at ~10 Hz
- Auto-restarts nvargus-daemon + re-execs process on TIMEOUT (os.execv)

### 1.6 Alternative: USB Camera (OV2311 — recommended if Argus unstable)
```bash
sudo apt-get install -y ros-noetic-usb-cam
rosrun usb_cam usb_cam_node \
  _video_device:=/dev/video0 \
  _image_width:=1280 _image_height:=960 \
  _pixel_format:=grey _framerate:=20 \
  _camera_name:=camera
```
Global shutter eliminates rolling-shutter distortion during Kalibr motion.
Remap `/usb_cam/image_raw` → `/camera/image_raw` for compatibility.

Verify:
```bash
rostopic hz /camera/image_raw/compressed   # expect ~9.9 Hz, stable
```

---

## Step 2 — Camera Intrinsic Calibration (IMX219)

**Status**: ✅ Done — `calibration/imx219_intrinsics.yaml`

### 2.0 Camera choice (read this first)

**IMX219 (CSI):** JP5.1.3 Argus has ISP IVC channel stability issues. sensor-mode=2
(1920×1080@30fps) is the stable mode. Calibration results are valid (rolling shutter
error is small at 10fps with slow motion).

**OV2311 (USB, recommended):** Global shutter, plug-and-play, no Argus.
Better for calibration — no rolling shutter. Use if IMX219/Argus is unstable.

### 2.1 Prerequisites
```bash
# In the Docker container
sudo apt-get install -y ros-noetic-camera-calibration
```

### 2.2 Start IMX219 publisher (terminal 1 — Jetson host, etro user)
```bash
# Wrapper restarts on crash and sets ROS env
bash /home/etro/vxs_ws/catkin_ws/src/vxs_sensor_ros1/scripts/run_imx219.sh
# Publishes:
#   /camera/image_raw            (1280x720 BGR @ 10 Hz)
#   /camera/image_raw/compressed (JPEG @ 10 Hz)
```

Verify on WSL or in the container:
```bash
rostopic hz /camera/image_raw/compressed   # should be ~10 Hz, stable
```

### 2.3 Run the calibrator (terminal 2 — WSL)

Use a checkerboard, NOT AprilGrid (the ROS calibrator wants chessboard corners).
Print a chessboard with `9x8` interior corners and **measure one square** with a ruler
(e.g. 50 mm = 0.05 m). Adjust `--size` and `--square` to match your printed target.

```bash
# On WSL (ROS_MASTER_URI already pointing at Jetson)
source /opt/ros/noetic/setup.bash
rosrun camera_calibration cameracalibrator.py \
  --size 9x8 \
  --square 0.05 \
  --no-service-check \
  image:=/camera/image_raw \
  camera:=/camera
```

Move the **checkerboard** (camera fixed) — or move the camera if the rig is mounted —
to cover X / Y / Size / Skew bars. When all four are green, click **CALIBRATE**, wait,
then **SAVE**.

Output: `/tmp/calibrationdata.tar.gz` → extract `ost.yaml`.
Save the result as:
```bash
# Copy back to the Jetson
scp /tmp/ost.yaml vxs@192.168.0.182:/home/vxs/vxs_ws/catkin_ws/src/vxs_sensor_ros1/calibration/imx219_intrinsics.yaml
```

### 2.4 Verify the result
```bash
# In the container
rosrun image_proc image_proc image:=/camera/image_raw   # publishes /camera/image_rect_color
# View the rectified image in RViz/rqt to confirm distortion is removed
```

**Trust webcam intrinsics** (640×480, kept for reference only):
```
fx=811.84, fy=811.33, cx=275.17, cy=270.57
k1=0.2076, k2=-0.6510, p1=0.0008, p2=-0.0027
```

---

## Step 3 — Camera ↔ IMU Calibration (Kalibr, using VN-100)

> **Why VN-100 instead of VXS built-in IMU?**
> The VXS IIM-42652 firmware does not provide valid hardware timestamps (`vxIMU.timestamp`
> is always zero on SDK 2.8). Kalibr requires accurate IMU timestamps. The VN-100 is a
> known-good external IMU with a reliable ROS driver and correct timestamps. Once the VXS
> firmware is fixed, this step can be redone with the built-in IMU.

### 3.1 Kalibr — ARM64 Docker container (on Jetson)

Kalibr runs in a **separate** ARM64 Docker container (`Dockerfile_arm64` in the repo),
NOT in the VXS SDK container and NOT in WSL (amd64/arm64 mismatch).

```bash
# Build Kalibr image (one-time, ~20 min)
cd /home/etro/vxs_ws/catkin_ws/src/vxs_sensor_ros1/docker
docker build -f Dockerfile_arm64 -t kalibr_arm64 .

# Run interactively with bag/calibration files mounted
docker run --rm -it \
  -v /home/etro/vxs_ws/catkin_ws/src/vxs_sensor_ros1/record:/data/bags \
  -v /home/etro/vxs_ws/catkin_ws/src/vxs_sensor_ros1/calibration:/data/calib \
  kalibr_arm64 bash
```

### 3.2 Target Config

The **checkerboard** (same 9×8, 5cm target used for intrinsics) works for Kalibr too.
Config already saved at `calibration/kalibr_target.yaml`:
```yaml
target_type: 'checkerboard'
tagCols: 9
tagRows: 8
rowSpacingMeters: 0.05
colSpacingMeters: 0.05
```

### 3.3 VN-100 Driver — Build and Run (inside VXS SDK container)

The `ros-noetic-vectornav` apt package does **not** exist for arm64. Build from source inside
the container (`agitated_mahavira`).

**One-time build (run once per container):**
```bash
# Install dependency first
sudo apt-get install -y libspdlog-dev

# Remove CATKIN_IGNORE so catkin sees the package
rm /home/vxs/vxs_ws/catkin_ws/src/vectornav/CATKIN_IGNORE

# Build into a separate space to avoid conflicts with host build artifacts
source /opt/ros/noetic/setup.bash
cd /home/vxs/vxs_ws/catkin_ws
catkin config --build-space build_container --devel-space devel_container \
  --cmake-args -DCMAKE_POLICY_VERSION_MINIMUM=3.5
catkin build vectornav_driver
```

> **Why separate build space?** The host (`etro` user) built packages into `build_host/` and
> `devel_host/` with hardcoded `/home/etro/` paths. Reusing those inside the container
> (where the user is `vxs`) causes cmake path mismatches.

**Start the driver:**
```bash
source /opt/ros/noetic/setup.bash
source /home/vxs/vxs_ws/catkin_ws/devel_container/setup.bash
export ROS_MASTER_URI=http://192.168.0.182:11311
export ROS_IP=192.168.0.182
export ROS_HOSTNAME=192.168.0.182

roslaunch vectornav_driver vectornav_driver_ros1.launch port:=/dev/ttyUSB0 baud_rate:=921600
# Publishes: /vectornav/imu/data at 200 Hz
```

**Verify (second terminal in container):**
```bash
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.0.182:11311
export ROS_IP=192.168.0.182
rostopic hz /vectornav/imu/data    # expect ~200 Hz
```

**VN-100 noise parameters** — datasheet values in `calibration/kalibr_imu.yaml` (use until
Allan variance is measured from a 30-min static bag):
```yaml
rostopic: /vectornav/imu/data
update_rate: 200.0
accelerometer_noise_density: 0.003924
accelerometer_random_walk:   0.000392
gyroscope_noise_density:     0.00029088
gyroscope_random_walk:       0.0000291
```

> **Note:** Topic is `/vectornav/imu/data`, not `/imu`. Kalibr bag must record this topic.

### 3.4 Rig Setup
Mount the VN-100 and the IMX219 camera **rigidly together** — they must not move relative
to each other during the calibration bag recording. Fix both to the same board or frame.
The relative pose between VN-100 and camera is what Kalibr will estimate.

### 3.5 Record the calibration bag (one command)

Use the session script — it handles roscore, camera publisher, VN-100 driver, and recording:

```bash
# On Jetson host (etro user) — starts everything and records for 120s
bash /home/etro/vxs_ws/catkin_ws/src/vxs_sensor_ros1/scripts/start_kalibr_bag_session.sh

# Custom duration (e.g. 180s) or unlimited (0):
bash start_kalibr_bag_session.sh cam_imu_calib_vn100 180
```

The script records: `/camera/image_raw` + `/camera/camera_info` + `/vectornav/imu/data`
Output: `record/cam_imu_calib_vn100.bag`

**Prerequisites:**
- VN-100 plugged into `/dev/ttyUSB0`
- Docker container `agitated_mahavira` running: `docker start agitated_mahavira`

**Motion requirements:**
- Move the rig (camera + VN-100 rigidly mounted together), keep checkerboard fixed
- Cover all three rotation axes — figure-8 or slow arc motions
- Slow, smooth — avoid jerks (IMX219 is rolling shutter)
- Keep checkerboard fully visible the whole time

### 3.6 Run Kalibr Camera-IMU Calibration

```bash
# Inside Kalibr ARM64 container (files are mounted at /data/)
docker run --rm -it \
  -v /home/etro/vxs_ws/catkin_ws/src/vxs_sensor_ros1/record:/data/bags \
  -v /home/etro/vxs_ws/catkin_ws/src/vxs_sensor_ros1/calibration:/data/calib \
  kalibr_arm64 bash

# Inside the container:
kalibr_calibrate_imu_camera \
  --bag /data/bags/cam_imu_calib_vn100.bag \
  --target /data/calib/kalibr_target.yaml \
  --imu /data/calib/kalibr_imu.yaml \
  --imu-models calibrated \
  --cams /data/calib/kalibr_camera_chain.yaml \
  --show-extraction
```

> **Note:** `kalibr_imu.yaml` uses datasheet noise values for now.
> Refine with Allan variance from `imu_static_30min.bag` before final calibration.

Output: `cam_imu_calib_vn100-camchain.yaml` — contains `T_C_I` (camera ← VN-100).

---

## Step 4 — VXS Point Cloud ↔ Camera Calibration (Dataset D)

### 4.1 Setup
- Keep rig fixed, move AprilGrid board to different poses
- For each pose: accumulate 50-100 VXS frames, capture camera image
- Target distance: 0.5m to 1.8m from sensor

### 4.2 Detect Board in Camera
- Use Kalibr or OpenCV AprilTag detector
- Get board pose `T_C_Bk` for each static pose k

### 4.3 Fit Plane in VXS Point Cloud
```bash
python3 vxs_sensor_ros1/offline_quality/bag_to_accumulated_ply.py \
  --bag record/dataset_d.bag \
  --out /tmp/dataset_d_frames \
  --acc-frames 2 \
  --min-range 0.10 --max-range 2.0
```

### 4.4 Optimize T_VXS_C
Minimize point-to-plane residuals:
```
r_i = n_B^T * (inv(T_C_Bk) * inv(T_VXS_C) * p_VXS) + d_B
```

---

## Step 5 — Chain Transforms

```python
import numpy as np

# Load from Kalibr output
T_C_I   = ...  # camera ← VXS IMU (4x4)
# Load from target calibration
T_VXS_C = ...  # VXS ← camera (4x4)

# Chain
T_VXS_I = T_VXS_C @ T_C_I

# Save
np.save("calibration/T_VXS_I.npy", T_VXS_I)
```

---

## Step 6 — Validation

```bash
# Project VXS points into camera image
# Board edges in VXS cloud should align with board edges in camera image
# Test at multiple distances (0.5m, 1.0m, 1.5m)
```

Typical failure signs:
- Good center, bad edges → distortion issue
- Constant offset → translation error
- Rotation mismatch → rotation extrinsic error
- Good static, bad motion → time offset issue

---

## ROS Network Setup

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

### WSL Laptop (`vxs` user, `192.168.0.164`)
```bash
# ~/.bashrc
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://192.168.0.182:11311
export ROS_IP=192.168.0.164
export ROS_HOSTNAME=192.168.0.164
# Requires WSL mirrored networking: C:\Users\rasra\.wslconfig → networkingMode=mirrored
```

---

## Key File Locations

| File | Description |
|------|-------------|
| `calibration/imx219_camera_info.yaml` | IMX219 intrinsics — ROS format (use with publisher) |
| `calibration/kalibr_camera_chain.yaml` | IMX219 intrinsics — Kalibr format (use with kalibr_calibrate_imu_camera) |
| `calibration/kalibr_imu.yaml` | VN-100 noise params for Kalibr (datasheet values — refine with Allan variance) |
| `calibration/kalibr_target.yaml` | Checkerboard target config for Kalibr (9×8, 5cm) |
| `calibration/vn100_kalibr.yaml` | VN-100 measured noise params (Allan variance — use once available) |
| `calibration/vxs_imu_kalibr.yaml` | VXS IIM-42652 noise params — **on hold** (firmware timestamp bug) |
| `calibration/webcam_intrinsics.yaml` | Trust webcam intrinsics (640×480, legacy reference) |
| `record/vn100_allan_static_30min.bag` | VN-100 static bag for Allan variance |
| `scripts/imx219_publisher.py` | IMX219 publisher (sensor-mode=4, 1280×720@10Hz, fakesink handoff) |
| `scripts/start_kalibr_bag_session.sh` | One-command bag recorder (camera + VN-100 + rosbag) |
| `launch/imx219.launch` | gscam-based launch (DEPRECATED — gscam segfaults on NVMM, use the script) |
| `launch/webcam.launch` | Trust webcam launch (container) |
| `launch/vxs_events.launch` | VXS sensor node (publish_imu:=true for IMU) |

---

## Current Status

| Step | Status |
|------|--------|
| IMX219 driver install | ✅ Done (Seeed J401, sensor-mode=2, module1 DTB patch) |
| IMX219 ROS streaming | ✅ Done (1280×720 @ 10 Hz, sensor-mode=2 stable) |
| IMX219 intrinsics | ✅ Done (1280×720, fx=1287.6, fy=1289.3, cx=646.8, cy=353.1) |
| OV2311 USB camera | ⏳ TODO — recommended alternative if Argus unstable |
| VN-100 driver + noise params | ✅ Done (200 Hz, datasheet values in `kalibr_imu.yaml`) |
| VN-100 Allan variance refinement | ⏳ TODO — record `vn100_static_30min.bag`, compute, update `kalibr_imu.yaml` |
| Kalibr ARM64 Docker image | ⏳ TODO — build `Dockerfile_arm64` |
| Camera-IMU bag recording | ⏳ TODO — run `start_kalibr_bag_session.sh` |
| Kalibr Camera-IMU calibration | ⏳ TODO |
| VXS-Camera calibration | ⏳ TODO |
| Transform chaining | ⏳ TODO |
| Validation | ⏳ TODO |
| VXS IMU (IIM-42652) | ⏳ On hold — firmware bug: timestamp always zero (reported to company) |
| Migration to Orin Nano Super | ⏳ TODO — see Migration section below |

---

## Migration to Jetson Orin Nano Super

The IMX219 works correctly on Jetson Orin Nano Super (JetPack 6.x / L4T 36.x).
Camera calibration and the full pipeline should be done there instead.

### Step 1 — Restore DTB on Orin NX (before moving hardware)

The Orin NX DTB was patched. If the Argus ISP IVC issue persists, restore
the original DTB before decommissioning:
```bash
sudo cp /boot/kernel_tegra234-p3767-0000-p3509-a02-user-custom.dtb.bak_20260512_160449 \
        /boot/kernel_tegra234-p3767-0000-p3509-a02-user-custom.dtb
sudo reboot
```

### Step 2 — Export the Docker container as an image

On the Orin NX, commit the running container and save to a tarball:
```bash
# Find the container name/ID (vxs SDK container)
docker ps -a

# Commit the running state to a new image
docker commit <container_name_or_id> vxs_ros1_noetic:orin_nx_backup

# Save to a tar file (place on external drive or NVMe)
docker save vxs_ros1_noetic:orin_nx_backup | gzip > ~/vxs_ros1_noetic_backup.tar.gz
# Expect ~2-4 GB, takes several minutes
```

### Step 3 — Transfer to Orin Nano Super

```bash
# Option A: SCP over network
scp ~/vxs_ros1_noetic_backup.tar.gz <user>@<orin_nano_ip>:~/

# Option B: USB drive (faster for large images)
cp ~/vxs_ros1_noetic_backup.tar.gz /media/usb/
# then plug into Orin Nano Super and copy

# Load on Orin Nano Super
docker load < ~/vxs_ros1_noetic_backup.tar.gz
docker images   # verify vxs_ros1_noetic:orin_nx_backup appears
```

### Step 4 — Run on Orin Nano Super

The `run_arm64.sh` script uses `$USER` so it works for any username:
```bash
# On Orin Nano Super host
bash /home/<user>/vxs_ws/catkin_ws/src/vxs_sensor_ros1/docker/run_arm64.sh \
  vxs_ros1_noetic:orin_nx_backup
```

The workspace is volume-mounted from `/home/$USER/vxs_ws`, so all
calibration files, scripts, and bags are available inside the container
without rebuilding the image.

### Step 5 — IMX219 on Orin Nano Super (JetPack 6.x)

JetPack 6.x (L4T 36.x) has the ISP IVC fix included — Argus is stable.
Sensor-mode=4 (1280×720@60fps) and sensor-mode=2 (1920×1080@30fps) both work.

DTB configuration:
```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
# Select "Camera IMX219 Dual" (if using single camera with same Dual overlay)
# OR select the appropriate single-camera overlay if available on JP6.x
```

Test:
```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 sensor-mode=2 num-buffers=30 \
  ! 'video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1' \
  ! fakesink sync=false
# Should complete 30 buffers without TIMEOUT
```

If Argus is stable, the module1 DTB patch may not be needed. Test without it first.

### Step 6 — Redo Camera Calibration on Orin Nano Super

The existing intrinsics (fx=1287.6, fy=1289.3) are valid if the same camera
and resolution are used. Redo calibration if:
- A different camera is used (e.g. OV2311)
- Resolution changes

Kalibr camera-IMU calibration must be redone on the Orin Nano Super because:
- The physical IMU-camera rigid mount may have changed during transfer
- JP6.x pipeline timing differs slightly from JP5.1.3

### Step 7 — Push code to GitHub

```bash
cd /home/vxs/vxs_ws/catkin_ws/src/vxs_sensor_ros1

# Stage all new files (calibration, scripts, docker, docs)
git add \
  IMX219_Camera_ROS_Install.txt \
  calibration/CALIBRATION_PIPELINE.md \
  calibration/imx219_camera_info.yaml \
  calibration/kalibr_camera_chain.yaml \
  calibration/kalibr_imu.yaml \
  calibration/kalibr_target.yaml \
  calibration/aprilgrid_6x6.yaml \
  scripts/imx219_publisher.py \
  scripts/start_kalibr_bag_session.sh \
  docker/run_arm64.sh \
  .gitignore

# Stage modified source files
git add CMakeLists.txt docker/run_arm64.sh launch/ src/

git commit -m "Add IMX219 publisher, calibration pipeline docs, VN-100 support

- imx219_publisher.py: sensor-mode=2, fakesink+handoffs, os.execv recovery
- CALIBRATION_PIPELINE.md: full step-by-step pipeline with VN-100, Kalibr ARM64
- IMX219_Camera_ROS_Install.txt: JP5.1.3 Argus issues, DTB patch, OV2311 alternative
- calibration/: intrinsics, Kalibr configs, AprilGrid and checkerboard targets
- docker/run_arm64.sh: XDG_RUNTIME_DIR, supplemental groups for GPU/video access
- .gitignore: exclude bags, PLY outputs, pycache"

git push origin main   # or your branch name
```

# Offline Pointcloud Quality Workflow

This folder is for **offline quality analysis** of VoxelSensor event clouds, separate from runtime launch files.

## Why this helps
- Compare raw vs filtered point clouds frame-by-frame.
- Quantify how many points each filter set removes.
- Pick LI-Init replay filter parameters with evidence instead of trial-and-error.

## 1) Export bag frames to PLY (raw + filtered)

```bash
cd /home/vxs/vxs_ws/catkin_ws/src/vxs_sensor_ros1/offline_quality
python3 bag_to_ply_events.py \
  --bag /home/vxs/vxs_ws/catkin_ws/src/vxs_sensor_ros1/record/vxs_li_init_calib_3min_20260409_172649.bag \
  --topic /vxs/pcloud/events \
  --out-dir /home/vxs/vxs_ws/catkin_ws/src/vxs_sensor_ros1/offline_quality/out_safe \
  --stride 10 \
  --max-range 2.0 \
  --voxel-size 0.0 \
  --ror-radius 0.0 \
  --ror-min-neighbors 0 \
  --sor-k 0
```

Outputs:
- `out_safe/raw/*.ply`
- `out_safe/filtered/*.ply`
- `out_safe/frame_stats.csv`

## 2) Try a more aggressive profile and compare

```bash
python3 bag_to_ply_events.py \
  --bag /home/vxs/vxs_ws/catkin_ws/src/vxs_sensor_ros1/record/vxs_li_init_calib_3min_20260409_172649.bag \
  --topic /vxs/pcloud/events \
  --out-dir /home/vxs/vxs_ws/catkin_ws/src/vxs_sensor_ros1/offline_quality/out_aggressive \
  --stride 10 \
  --max-range 1.2 \
  --voxel-size 0.004 \
  --ror-radius 0.02 \
  --ror-min-neighbors 8 \
  --sor-k 10 \
  --sor-std-ratio 1.0
```

Compare `frame_stats.csv` retention ratio between profiles.

## Suggested separation for LI-Init experiments

Keep production vs experiments separate by files, not by copying the full package:

- Production replay launch:
  - `launch/vxs_li_init_replay.launch`
- Experimental replay launch:
  - `launch/vxs_li_init_replay_exp.launch`
- Production config:
  - `LiDAR_IMU_Init/config/vxs_sensor.yaml`
- Experimental config:
  - `LiDAR_IMU_Init/config/vxs_sensor_exp.yaml`

This avoids code divergence while allowing safe tuning.

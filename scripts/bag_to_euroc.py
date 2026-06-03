#!/usr/bin/env python3
"""
Convert a ROS bag to EuRoC/Basalt dataset format.

Output structure:
  <out_dir>/
    mav0/
      cam0/
        data/           <- PNG images named by timestamp in nanoseconds
        data.csv        <- timestamp [ns], filename
        sensor.yaml     <- camera params (filled from --cam yaml)
      imu0/
        data.csv        <- timestamp [ns], gx, gy, gz, ax, ay, az
        sensor.yaml     <- IMU params (filled from --imu yaml)

Usage:
    python3 bag_to_euroc.py \
        --bag record/cam_imu_calib_ov9281_vn100_restamped.bag \
        --cam-topic /camera/image_raw \
        --imu-topic /vectornav/imu/data \
        --cam calibration/ov9281_camera_info.yaml \
        --imu calibration/vn100_kalibr.yaml \
        --out /tmp/vxs_euroc
"""

import argparse, os, csv, yaml
from pathlib import Path
import numpy as np
import cv2
import rosbag
from sensor_msgs import point_cloud2

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--bag",       required=True)
    p.add_argument("--cam-topic", default="/camera/image_raw")
    p.add_argument("--imu-topic", default="/vectornav/imu/data")
    p.add_argument("--cam",       required=True, help="ov9281_camera_info.yaml")
    p.add_argument("--imu",       required=True, help="vn100_kalibr.yaml")
    p.add_argument("--out",       required=True, help="output EuRoC directory")
    return p.parse_args()


def write_cam_sensor_yaml(path, cam_yaml):
    with open(cam_yaml) as f:
        d = yaml.safe_load(f)
    K = d["camera_matrix"]["data"]
    D = d["distortion_coefficients"]["data"]
    sensor = {
        "sensor_type": "camera",
        "comment": "OV9281 global shutter USB camera",
        "T_BS": {
            "cols": 4, "rows": 4,
            "data": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
        },
        "rate_hz": 20,
        "resolution": [d["image_width"], d["image_height"]],
        "camera_model": "pinhole",
        "intrinsics": [K[0], K[4], K[2], K[5]],  # fx fy cx cy
        "distortion_model": "radtan",
        "distortion_coefficients": D[:4],
    }
    with open(path, "w") as f:
        yaml.dump(sensor, f, default_flow_style=False)


def write_imu_sensor_yaml(path, imu_yaml):
    with open(imu_yaml) as f:
        d = yaml.safe_load(f)
    sensor = {
        "sensor_type": "imu",
        "comment": "VectorNav VN-100",
        "T_BS": {
            "cols": 4, "rows": 4,
            "data": [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1]
        },
        "rate_hz": int(d.get("update_rate", 200)),
        "gyroscope_noise_density":     d["gyroscope_noise_density"],
        "gyroscope_random_walk":       d["gyroscope_random_walk"],
        "accelerometer_noise_density": d["accelerometer_noise_density"],
        "accelerometer_random_walk":   d["accelerometer_random_walk"],
    }
    with open(path, "w") as f:
        yaml.dump(sensor, f, default_flow_style=False)


def main():
    args = parse_args()

    # Create output dirs
    cam_dir = Path(args.out) / "mav0" / "cam0" / "data"
    imu_dir = Path(args.out) / "mav0" / "imu0"
    cam_dir.mkdir(parents=True, exist_ok=True)
    imu_dir.mkdir(parents=True, exist_ok=True)

    # Write sensor yamls
    write_cam_sensor_yaml(Path(args.out) / "mav0" / "cam0" / "sensor.yaml", args.cam)
    write_imu_sensor_yaml(Path(args.out) / "mav0" / "imu0" / "sensor.yaml", args.imu)

    print(f"Reading bag: {args.bag}")
    bag = rosbag.Bag(args.bag)
    info = bag.get_type_and_topic_info().topics
    print(f"  Topics: {list(info.keys())}")

    # ── Camera frames ─────────────────────────────────────────────────────────
    print(f"Extracting camera frames from {args.cam_topic}...")
    cam_csv_rows = []
    n_cam = 0
    for _, msg, t in bag.read_messages(topics=[args.cam_topic]):
        ts_ns = t.to_nsec()
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        enc = msg.encoding
        if enc in ("bgr8", "rgb8"):
            img = arr.reshape(msg.height, msg.width, 3).copy()
            if enc == "rgb8":
                img = img[:, :, ::-1]
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        elif enc == "mono8":
            gray = arr.reshape(msg.height, msg.width).copy()
        elif enc == "bgra8":
            img = arr.reshape(msg.height, msg.width, 4)[:, :, :3].copy()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            # Try MJPEG decode
            gray = cv2.imdecode(arr, cv2.IMREAD_GRAYSCALE)
            if gray is None:
                continue

        fname = f"{ts_ns}.png"
        cv2.imwrite(str(cam_dir / fname), gray)
        cam_csv_rows.append([ts_ns, fname])
        n_cam += 1
        if n_cam % 100 == 0:
            print(f"  {n_cam} frames...", end="\r")

    print(f"\n  Extracted {n_cam} camera frames")

    with open(Path(args.out) / "mav0" / "cam0" / "data.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["#timestamp [ns]", "filename"])
        w.writerows(cam_csv_rows)

    # ── IMU data ──────────────────────────────────────────────────────────────
    print(f"Extracting IMU data from {args.imu_topic}...")
    imu_rows = []
    for _, msg, t in bag.read_messages(topics=[args.imu_topic]):
        ts_ns = t.to_nsec()
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        imu_rows.append([ts_ns, gx, gy, gz, ax, ay, az])

    print(f"  Extracted {len(imu_rows)} IMU samples")
    bag.close()

    with open(imu_dir / "data.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["#timestamp [ns]", "w_RS_S_x [rad s^-1]", "w_RS_S_y [rad s^-1]",
                    "w_RS_S_z [rad s^-1]", "a_RS_S_x [m s^-2]",
                    "a_RS_S_y [m s^-2]", "a_RS_S_z [m s^-2]"])
        w.writerows(imu_rows)

    print(f"\nDone. EuRoC dataset written to: {args.out}")
    print(f"  cam0: {n_cam} frames  →  {args.out}/mav0/cam0/")
    print(f"  imu0: {len(imu_rows)} samples →  {args.out}/mav0/imu0/")
    print(f"\nCopy to PC and run Basalt:")
    print(f"  basalt_calibrate --dataset-path {args.out} --dataset-type euroc \\")
    print(f"    --aprilgrid ~/Projects/basalt/data/aprilgrid_6x6.json")


if __name__ == "__main__":
    main()

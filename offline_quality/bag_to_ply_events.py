#!/usr/bin/env python3
"""
Export PointCloud2 frames from a rosbag to PLY and evaluate filter impact.

Saves:
  - raw PLY frames
  - filtered PLY frames
  - CSV stats per frame (raw points, filtered points, retention ratio)

Filter stages mirror pointcloud_events_to_velodyne.py:
  1) max range
  2) voxel downsample
  3) radius outlier removal (ROR)
  4) statistical outlier removal (SOR)
"""

import argparse
import csv
import os
from typing import Tuple

import numpy as np
import rosbag
from sensor_msgs import point_cloud2


def range_filter(pts: np.ndarray, max_range: float) -> np.ndarray:
    if max_range <= 0.0 or len(pts) == 0:
        return pts
    radii = np.linalg.norm(pts[:, :3], axis=1)
    return pts[radii <= max_range]


def voxel_downsample(pts: np.ndarray, voxel_size: float) -> np.ndarray:
    if voxel_size <= 0.0 or len(pts) == 0:
        return pts
    coords = pts[:, :3]
    vox = np.floor(coords / voxel_size).astype(np.int32)
    keys = vox[:, 0].astype(np.int64) * 1_000_003 + vox[:, 1].astype(np.int64) * 1_009 + vox[:, 2].astype(np.int64)
    order = np.argsort(keys)
    _, first = np.unique(keys[order], return_index=True)
    return pts[order][first]


def radius_outlier_removal(pts: np.ndarray, radius: float, min_neighbors: int) -> np.ndarray:
    if radius <= 0.0 or min_neighbors <= 0 or len(pts) <= min_neighbors + 1:
        return pts
    from scipy.spatial import cKDTree

    tree = cKDTree(pts[:, :3])
    counts = tree.query_ball_point(pts[:, :3], r=radius, return_length=True)
    return pts[counts > min_neighbors]


def statistical_outlier_removal(pts: np.ndarray, k: int, std_ratio: float) -> np.ndarray:
    if k <= 0 or len(pts) <= k + 1:
        return pts
    from scipy.spatial import cKDTree

    tree = cKDTree(pts[:, :3])
    dists, _ = tree.query(pts[:, :3], k=k + 1)
    mean_dist = dists[:, 1:].mean(axis=1)
    thr = mean_dist.mean() + std_ratio * mean_dist.std()
    return pts[mean_dist < thr]


def write_ply_xyz(path: str, xyz: np.ndarray) -> None:
    with open(path, "w", encoding="ascii") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(xyz)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("end_header\n")
        for p in xyz:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


def apply_filters(pts: np.ndarray, args: argparse.Namespace) -> np.ndarray:
    out = range_filter(pts, args.max_range)
    out = voxel_downsample(out, args.voxel_size)
    out = radius_outlier_removal(out, args.ror_radius, args.ror_min_neighbors)
    out = statistical_outlier_removal(out, args.sor_k, args.sor_std_ratio)
    return out


def read_xyz_from_msg(msg) -> np.ndarray:
    rows = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if not rows:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(rows, dtype=np.float64)


def ensure_dirs(base_out: str) -> Tuple[str, str]:
    raw_dir = os.path.join(base_out, "raw")
    fil_dir = os.path.join(base_out, "filtered")
    os.makedirs(raw_dir, exist_ok=True)
    os.makedirs(fil_dir, exist_ok=True)
    return raw_dir, fil_dir


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Export rosbag PointCloud2 to PLY and filter stats.")
    p.add_argument("--bag", required=True, help="Input .bag file")
    p.add_argument("--topic", default="/vxs/pcloud/events", help="PointCloud2 topic")
    p.add_argument("--out-dir", required=True, help="Output folder")
    p.add_argument("--stride", type=int, default=1, help="Save every Nth frame")
    p.add_argument("--max-frames", type=int, default=0, help="0=all, otherwise stop after N saved frames")

    p.add_argument("--max-range", type=float, default=2.0)
    p.add_argument("--voxel-size", type=float, default=0.0)
    p.add_argument("--ror-radius", type=float, default=0.0)
    p.add_argument("--ror-min-neighbors", type=int, default=0)
    p.add_argument("--sor-k", type=int, default=0)
    p.add_argument("--sor-std-ratio", type=float, default=1.0)
    return p.parse_args()


def main() -> None:
    args = parse_args()
    raw_dir, fil_dir = ensure_dirs(args.out_dir)
    stats_csv = os.path.join(args.out_dir, "frame_stats.csv")

    saved = 0
    seen = 0
    with rosbag.Bag(args.bag, "r") as bag, open(stats_csv, "w", newline="", encoding="utf-8") as fcsv:
        writer = csv.writer(fcsv)
        writer.writerow(["frame_idx", "bag_time", "raw_points", "filtered_points", "retention_ratio"])

        for _, msg, t in bag.read_messages(topics=[args.topic]):
            seen += 1
            if args.stride > 1 and ((seen - 1) % args.stride != 0):
                continue

            raw_xyz = read_xyz_from_msg(msg)
            raw_n = int(raw_xyz.shape[0])
            filtered_xyz = apply_filters(raw_xyz, args)
            fil_n = int(filtered_xyz.shape[0])
            ratio = (float(fil_n) / float(raw_n)) if raw_n > 0 else 0.0

            frame_name = f"frame_{saved:06d}.ply"
            write_ply_xyz(os.path.join(raw_dir, frame_name), raw_xyz)
            write_ply_xyz(os.path.join(fil_dir, frame_name), filtered_xyz)

            writer.writerow([saved, f"{t.to_sec():.9f}", raw_n, fil_n, f"{ratio:.6f}"])
            saved += 1
            if args.max_frames > 0 and saved >= args.max_frames:
                break

    print(f"Done. Seen={seen}, saved={saved}")
    print(f"Raw PLY:      {raw_dir}")
    print(f"Filtered PLY: {fil_dir}")
    print(f"Stats CSV:    {stats_csv}")


if __name__ == "__main__":
    main()

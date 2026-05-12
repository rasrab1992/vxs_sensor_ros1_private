#!/usr/bin/env python3
"""
Extract accumulated virtual scans from a rosbag as PLY files.

Accumulates --acc-frames consecutive raw frames into one virtual scan
(covering a full Lissajous period), then saves it as a PLY file.
Output is a directory of frame_000000.ply, frame_000001.ply, ... ready
for the turntable-registration pipeline.

Usage:
  python3 bag_to_accumulated_ply.py \
      --bag record/icp_param_test.bag \
      --out /tmp/icp_frames \
      --acc-frames 5 \
      --min-range 0.10 \
      --max-range 3.0
"""

from __future__ import annotations

import argparse
import os

import numpy as np
import rosbag
from sensor_msgs import point_cloud2


def read_xyz(msg) -> np.ndarray:
    rows = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if not rows:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(rows, dtype=np.float64)


def range_filter(pts: np.ndarray, lo: float, hi: float) -> np.ndarray:
    if len(pts) == 0:
        return pts
    r = np.linalg.norm(pts, axis=1)
    mask = np.ones(len(pts), dtype=bool)
    if lo > 0:
        mask &= r >= lo
    if hi > 0:
        mask &= r <= hi
    return pts[mask]


def write_ply(path: str, pts: np.ndarray) -> None:
    with open(path, "w", encoding="ascii") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
        for p in pts:
            f.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")


def parse_args():
    p = argparse.ArgumentParser(description="Export accumulated virtual scans from rosbag to PLY")
    p.add_argument("--bag",        required=True)
    p.add_argument("--out",        required=True)
    p.add_argument("--topic",      default="/vxs/pcloud/events")
    p.add_argument("--acc-frames", type=int, default=5,
                   help="Raw frames to accumulate per virtual scan (default: 5)")
    p.add_argument("--skip-frames", type=int, default=5,
                   help="Drop first N raw frames (warm-up)")
    p.add_argument("--min-range",  type=float, default=0.10)
    p.add_argument("--max-range",  type=float, default=3.0)
    p.add_argument("--min-points", type=int, default=100,
                   help="Skip virtual scans with fewer than this many points")
    p.add_argument("--max-scans",  type=int, default=0,
                   help="Stop after N virtual scans (0=all)")
    return p.parse_args()


def main():
    args = parse_args()
    os.makedirs(args.out, exist_ok=True)

    acc_buf: list[np.ndarray] = []
    raw_seen = vscan_idx = 0

    with rosbag.Bag(args.bag) as bag:
        total = bag.get_message_count(args.topic)
        print(f"Bag:   {args.bag}")
        print(f"Topic: {args.topic}  ({total} msgs)")
        print(f"Accumulating {args.acc_frames} raw frames → 1 virtual scan PLY")
        print(f"Output: {args.out}")
        print()

        for _, msg, _ in bag.read_messages(topics=[args.topic]):
            raw_seen += 1
            if raw_seen <= args.skip_frames:
                continue

            pts = range_filter(read_xyz(msg), args.min_range, args.max_range)
            if len(pts):
                acc_buf.append(pts)

            if len(acc_buf) < args.acc_frames:
                continue

            vscan = np.concatenate(acc_buf)
            acc_buf.clear()

            if len(vscan) < args.min_points:
                print(f"[{vscan_idx:06d}] skipped — only {len(vscan)} pts")
                continue

            fname = os.path.join(args.out, f"frame_{vscan_idx:06d}.ply")
            write_ply(fname, vscan)
            print(f"[{vscan_idx:06d}] {len(vscan):6d} pts  → {os.path.basename(fname)}")
            vscan_idx += 1

            if args.max_scans > 0 and vscan_idx >= args.max_scans:
                break

    print(f"\nDone: {vscan_idx} virtual scan PLYs in {args.out}")


if __name__ == "__main__":
    main()

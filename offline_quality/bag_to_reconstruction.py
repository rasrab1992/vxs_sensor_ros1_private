#!/usr/bin/env python3
"""
Extract /vxs/pcloud/events frames from a rosbag and run map-to-frame VGICP
to produce a fused reconstruction PLY.

Strategy: MAP-TO-FRAME (not frame-to-frame).
  - Accumulate N raw frames into one virtual scan (covers full Lissajous cycle).
  - Register the virtual scan against the GROWING MAP of all accepted scans.
  - Map provides much more overlap than the previous scan alone.
  - Use small_gicp IncrementalVoxelMap for fast nearest-neighbour lookup.

This directly fixes the Lissajous overlap problem:
  - Frame-to-frame GICP fails because consecutive frames cover different parts
    of the Lissajous pattern (near-zero overlap when sensor moves).
  - Map-to-frame GICP always has full overlap because the map accumulates all
    previously seen geometry.

Output:
  <out_dir>/fused.ply       final downsampled reconstruction
  <out_dir>/poses.npy       (M,4,4) world transforms
  <out_dir>/stats.csv       per-scan stats

Usage:
  python3 bag_to_reconstruction.py \\
      --bag record/vxs_li_init_calib_07_walking.bag \\
      --out out_07_mapframe \\
      --acc-frames 5
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time

import numpy as np
import rosbag
from sensor_msgs import point_cloud2
import small_gicp


# ---------------------------------------------------------------------------
# I/O
# ---------------------------------------------------------------------------

def read_xyz(msg) -> np.ndarray:
    rows = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if not rows:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(rows, dtype=np.float64)


def write_ply(path: str, pts: np.ndarray) -> None:
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {len(pts)}\n")
        f.write("property float x\nproperty float y\nproperty float z\nend_header\n")
        np.savetxt(f, pts[:, :3], fmt="%.6f")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def range_filter(pts: np.ndarray, lo: float, hi: float) -> np.ndarray:
    if len(pts) == 0:
        return pts
    r = np.linalg.norm(pts, axis=1)
    m = np.ones(len(pts), bool)
    if lo > 0: m &= r >= lo
    if hi > 0: m &= r <= hi
    return pts[m]


def voxel_ds(pts: np.ndarray, v: float) -> np.ndarray:
    if v <= 0 or len(pts) == 0:
        return pts
    return small_gicp.voxelgrid_sampling(
        small_gicp.PointCloud(pts.astype(np.float64)), v
    ).points()[:, :3]


def transform_pts(pts: np.ndarray, T: np.ndarray) -> np.ndarray:
    if len(pts) == 0:
        return pts
    return (T[:3, :3] @ pts.T).T + T[:3, 3]


def rot_angle_deg(T: np.ndarray) -> float:
    return math.degrees(math.acos(
        np.clip((np.trace(T[:3, :3]) - 1.0) / 2.0, -1.0, 1.0)))


def trans_m(T: np.ndarray) -> float:
    return float(np.linalg.norm(T[:3, 3]))


# ---------------------------------------------------------------------------
# Args
# ---------------------------------------------------------------------------

def parse_args():
    p = argparse.ArgumentParser(description="Map-to-frame VGICP reconstruction for Lissajous LiDAR")
    p.add_argument("--bag",    required=True)
    p.add_argument("--out",    required=True)
    p.add_argument("--topic",  default="/vxs/pcloud/events")
    p.add_argument("--skip-frames", type=int, default=10,
                   help="Drop first N raw frames (warm-up)")
    p.add_argument("--max-scans", type=int, default=0,
                   help="Stop after N virtual scans (0=all)")

    # Lissajous accumulation
    p.add_argument("--acc-frames", type=int, default=5,
                   help="Raw frames per virtual scan. ~1 full Lissajous period.")

    # Range filter
    p.add_argument("--min-range", type=float, default=0.10)
    p.add_argument("--max-range", type=float, default=5.0)

    # Per-scan voxel (applied before GICP, trades density for speed)
    p.add_argument("--scan-voxel", type=float, default=0.0,
                   help="Voxel applied to each virtual scan before GICP (0=off)")
    p.add_argument("--min-points", type=int, default=200)

    # GICP / VGICP params
    p.add_argument("--reg-type", default="VGICP",
                   choices=["ICP", "PLANE_ICP", "GICP", "VGICP"])
    p.add_argument("--downsample", type=float, default=0.05,
                   help="small_gicp internal downsampling resolution (m)")
    p.add_argument("--voxel-res",  type=float, default=0.10,
                   help="VGICP voxel resolution (m)")
    p.add_argument("--max-corr",   type=float, default=1.0,
                   help="Max correspondence distance (m). Larger = more forgiving on sparse clouds.")
    p.add_argument("--num-threads", type=int, default=4)
    p.add_argument("--max-iter",    type=int, default=30)

    # Plausibility gate
    p.add_argument("--max-trans", type=float, default=0.5,
                   help="Reject if Δt > this (m)")
    p.add_argument("--max-rot",   type=float, default=30.0,
                   help="Reject if Δθ > this (deg)")

    # Map management
    p.add_argument("--map-voxel", type=float, default=0.05,
                   help="Voxel size for the incremental map (m). Controls map density.")
    p.add_argument("--map-max-pts", type=int, default=0,
                   help="Downsample map to this many pts before GICP (0=off, slow on large maps)")

    # Output
    p.add_argument("--fused-voxel", type=float, default=0.03)
    return p.parse_args()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    args = parse_args()
    os.makedirs(args.out, exist_ok=True)

    fused_path = os.path.join(args.out, "fused.ply")
    poses_path = os.path.join(args.out, "poses.npy")
    stats_path = os.path.join(args.out, "stats.csv")

    # Accumulated map as a simple numpy array (voxel-managed)
    map_pts: np.ndarray = np.zeros((0, 3), dtype=np.float64)

    poses: list[np.ndarray] = []
    fused_batches: list[np.ndarray] = []
    cum_T = np.eye(4)
    last_guess = np.eye(4)

    acc_buf: list[np.ndarray] = []
    raw_seen = vscan_idx = skipped_pts = skipped_impl = 0

    t0 = time.perf_counter()

    with rosbag.Bag(args.bag) as bag, open(stats_path, "w", newline="") as fcsv:
        writer = csv.writer(fcsv)
        writer.writerow(["vscan", "bag_time", "scan_pts", "map_pts_before",
                         "trans_m", "rot_deg", "converged", "ms"])

        total = bag.get_message_count(args.topic)
        print(f"Bag:   {args.bag}")
        print(f"Topic: {args.topic}  ({total} msgs)")
        print(f"Accumulation: {args.acc_frames} raw frames → 1 virtual scan")
        print(f"Registration: {args.reg_type}  downsample={args.downsample}m  "
              f"max_corr={args.max_corr}m  map_voxel={args.map_voxel}m")
        print()

        for _, msg, bag_t in bag.read_messages(topics=[args.topic]):
            raw_seen += 1
            if raw_seen <= args.skip_frames:
                continue

            pts = range_filter(read_xyz(msg), args.min_range, args.max_range)
            if len(pts):
                acc_buf.append(pts)
            if len(acc_buf) < args.acc_frames:
                continue

            # Build virtual scan from accumulated frames
            vscan = np.concatenate(acc_buf)
            acc_buf.clear()
            if args.scan_voxel > 0:
                vscan = voxel_ds(vscan, args.scan_voxel)

            if len(vscan) < args.min_points:
                skipped_pts += 1
                continue

            # --- First scan: init map ---
            if len(map_pts) == 0:
                map_pts = voxel_ds(vscan, args.map_voxel)
                poses.append(cum_T.copy())
                fused_batches.append(vscan.copy())
                print(f"[{vscan_idx:04d}] reference scan: {len(vscan)} pts  map: {len(map_pts)} pts")
                writer.writerow([vscan_idx, f"{bag_t.to_sec():.6f}",
                                  len(vscan), 0, 0.0, 0.0, True, 0.0])
                vscan_idx += 1
                continue

            # --- Map-to-frame GICP ---
            # Target = current map, Source = new virtual scan (in sensor frame)
            # We need source in world frame for correspondence; init_T provides that.
            t_reg = time.perf_counter()

            # Optionally cap map size for speed
            target = map_pts
            if args.map_max_pts > 0 and len(target) > args.map_max_pts:
                idx = np.random.choice(len(target), args.map_max_pts, replace=False)
                target = target[idx]

            kwargs = dict(
                registration_type=args.reg_type,
                downsampling_resolution=args.downsample,
                max_correspondence_distance=args.max_corr,
                num_threads=args.num_threads,
                max_iterations=args.max_iter,
                init_T_target_source=last_guess,
            )
            if args.reg_type.upper() == "VGICP":
                kwargs["voxel_resolution"] = args.voxel_res

            try:
                # Source points are in sensor frame; align gives T_world_sensor
                result = small_gicp.align(
                    target.astype(np.float64),
                    vscan.astype(np.float64),
                    **kwargs,
                )
                rel_T = result.T_target_source
                converged = bool(result.converged)
            except Exception as e:
                print(f"[{vscan_idx:04d}] GICP exception: {e} — holding pose")
                poses.append(cum_T.copy())
                fused_batches.append(transform_pts(vscan, cum_T))
                vscan_idx += 1
                continue

            elapsed_ms = (time.perf_counter() - t_reg) * 1000.0
            dt = trans_m(rel_T)
            drot = rot_angle_deg(rel_T)

            if dt > args.max_trans or drot > args.max_rot:
                skipped_impl += 1
                print(f"[{vscan_idx:04d}] implausible Δt={dt*1000:.0f}mm Δθ={drot:.1f}° — holding")
                rel_T = np.eye(4)
                converged = False

            prev_T = cum_T.copy()
            cum_T = rel_T  # rel_T is T_map_sensor (absolute world pose)
            # Constant-velocity prediction: next pose ≈ current + last delta
            delta_T = np.linalg.inv(prev_T) @ cum_T
            last_guess = cum_T @ delta_T  # predict next frame
            poses.append(cum_T.copy())

            # Transform scan to world frame and add to map
            aligned = transform_pts(vscan, cum_T)
            fused_batches.append(aligned)

            # Grow map with new aligned points, keep map density controlled
            map_pts = voxel_ds(np.concatenate([map_pts, aligned]), args.map_voxel)

            print(f"[{vscan_idx:04d}] pts={len(vscan):5d}  "
                  f"Δt={dt*1000:6.1f}mm  Δθ={drot:5.2f}°  "
                  f"conv={converged}  map={len(map_pts):6d}  {elapsed_ms:.0f}ms")
            writer.writerow([vscan_idx, f"{bag_t.to_sec():.6f}",
                              len(vscan), len(map_pts),
                              f"{dt:.5f}", f"{drot:.4f}", converged, f"{elapsed_ms:.0f}"])
            vscan_idx += 1

            if args.max_scans > 0 and vscan_idx >= args.max_scans:
                break

    elapsed = time.perf_counter() - t0
    print(f"\nDone: {vscan_idx} scans in {elapsed:.1f}s  ({elapsed/max(vscan_idx,1)*1000:.0f}ms/scan)")
    print(f"Skipped: {skipped_pts} (pts)  {skipped_impl} (implausible)")

    if not fused_batches:
        print("No scans — check topic and params.")
        sys.exit(1)

    conv_count = sum(1 for r in open(stats_path).readlines()[1:] if ",True," in r)
    print(f"Converged: {conv_count}/{vscan_idx} ({100*conv_count/max(vscan_idx,1):.1f}%)")

    print("Building final fused cloud...")
    fused = np.concatenate(fused_batches)
    print(f"  Raw: {len(fused):,} pts")
    fused = voxel_ds(fused, args.fused_voxel)
    print(f"  After {args.fused_voxel}m voxel: {len(fused):,} pts")
    for i, ax in enumerate("XYZ"):
        print(f"  {ax}: [{fused[:,i].min():.2f}, {fused[:,i].max():.2f}] m")

    write_ply(fused_path, fused)
    np.save(poses_path, np.stack(poses))
    print(f"\nSaved: {fused_path}")
    print(f"Saved: {poses_path}")
    print(f"Saved: {stats_path}")


if __name__ == "__main__":
    main()

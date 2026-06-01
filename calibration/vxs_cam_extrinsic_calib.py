#!/usr/bin/env python3
"""
VXS LiDAR -> Camera extrinsic calibration via checkerboard plane matching.

For each static pose in the bag:
  - Detect checkerboard in camera image -> board pose T_cam_board via solvePnP
  - Fit plane to VXS points on the board surface via RANSAC
  - Optimize T_VXS_cam so camera-derived plane and VXS-fitted plane agree

Usage:
    python3 vxs_cam_extrinsic_calib.py \
        --bag record/vxs_cam_calib.bag \
        --cam calibration/ov9281_camera_info.yaml \
        --cols 9 --rows 8 --square 0.05 \
        --out calibration/T_vxs_cam.yaml \
        --acc-frames 5 \
        --min-range 0.3 --max-range 2.0

Requirements (all available in container):
    numpy, scipy, opencv-python, pyyaml, rosbag
"""

from __future__ import annotations
import argparse, sys, yaml
from pathlib import Path

import numpy as np
import cv2
import rosbag
from sensor_msgs import point_cloud2
from scipy.spatial.transform import Rotation
from scipy.optimize import minimize


# ── Camera utils ──────────────────────────────────────────────────────────────

def load_camera_info(path: str):
    with open(path) as f:
        d = yaml.safe_load(f)
    K = np.array(d["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    D = np.array(d["distortion_coefficients"]["data"], dtype=np.float64)
    return K, D


def checkerboard_obj_pts(cols: int, rows: int, square: float) -> np.ndarray:
    """3D corner positions in board frame (Z=0 plane)."""
    pts = np.zeros((cols * rows, 3), dtype=np.float64)
    for r in range(rows):
        for c in range(cols):
            pts[r * cols + c] = [c * square, r * square, 0.0]
    return pts


def detect_board(img_bgr: np.ndarray, K: np.ndarray, D: np.ndarray,
                 cols: int, rows: int, square: float):
    """
    Detect checkerboard and return (R_cam_board, t_cam_board) or None.
    """
    gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, (cols, rows), flags)
    if not found:
        return None

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    obj_pts = checkerboard_obj_pts(cols, rows, square)
    ok, rvec, tvec = cv2.solvePnP(obj_pts, corners, K, D,
                                   flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None

    R, _ = cv2.Rodrigues(rvec)
    return R, tvec.flatten()


def board_plane_in_cam(R_cb: np.ndarray, t_cb: np.ndarray):
    """
    Board plane in camera frame as (normal, d) where normal^T * p + d = 0.
    Normal always points toward camera origin (n @ t_cb < 0).
    """
    normal = R_cb[:, 2]
    # Ensure normal points toward camera (dot with board position < 0)
    if normal @ t_cb > 0:
        normal = -normal
    d = float(-(normal @ t_cb))
    return normal, d


# ── VXS plane fitting ─────────────────────────────────────────────────────────

def fit_plane_ransac(pts: np.ndarray, n_iter: int = 1000, thresh: float = 0.015,
                     dist_prior: float = None, dist_tol: float = 0.15):
    """
    Fit plane via RANSAC then refine with SVD on inliers.
    If dist_prior is given, pre-filter points to within dist_tol of that range.
    Returns (normal, d) where normal^T * p + d = 0, or None.
    """
    if dist_prior is not None:
        r = np.linalg.norm(pts, axis=1)
        pts = pts[np.abs(r - dist_prior) < dist_tol]

    if len(pts) < 10:
        return None

    best_inliers = 0
    best_n, best_d = None, None
    rng = np.random.default_rng(0)

    for _ in range(n_iter):
        idx = rng.choice(len(pts), 3, replace=False)
        p0, p1, p2 = pts[idx]
        n = np.cross(p1 - p0, p2 - p0)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n /= nn
        dists = np.abs(pts @ n - (n @ p0))
        inliers = int(np.sum(dists < thresh))
        if inliers > best_inliers:
            best_inliers = inliers
            best_n = n.copy()
            best_d = float(-(n @ p0))

    if best_n is None or best_inliers < 8:
        return None

    # SVD refinement on inliers
    dists = np.abs(pts @ best_n + best_d)
    inlier_pts = pts[dists < thresh]
    centroid = inlier_pts.mean(axis=0)
    _, _, Vt = np.linalg.svd(inlier_pts - centroid)
    n = Vt[-1]
    # Always point normal toward sensor origin (away from board)
    if n @ centroid > 0:
        n = -n
    d = float(-(n @ centroid))
    return n, d


# ── SE3 optimization ──────────────────────────────────────────────────────────

def T_from_params(params: np.ndarray) -> np.ndarray:
    """[rx,ry,rz, tx,ty,tz] -> 4x4 SE3 matrix."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(params[:3]).as_matrix()
    T[:3, 3] = params[3:]
    return T


def cost_fn(params, vxs_planes, cam_planes):
    """
    Minimize plane alignment error after applying T_VXS_cam.
    Both normals are consistently oriented toward their sensor origins,
    so we can use signed distances directly.
    """
    R = Rotation.from_rotvec(params[:3]).as_matrix()
    t = params[3:]
    total = 0.0
    for (n_v, d_v), (n_c, d_c) in zip(vxs_planes, cam_planes):
        # Transform camera plane normal+distance into VXS frame
        n_c_vxs = R @ n_c
        d_c_vxs = d_c - float(n_c_vxs @ t)
        # Normal alignment — both point toward their sensor, so dot should be +1
        cos_a = float(np.clip(n_v @ n_c_vxs, -1.0, 1.0))
        total += (1.0 - cos_a) * 500.0
        # Signed distance agreement
        total += (d_v - d_c_vxs) ** 2 * 500.0
    return total


# ── Bag reading ───────────────────────────────────────────────────────────────

def read_vxs_frames(bag, topic, acc, lo, hi):
    """Accumulate `acc` VXS frames into one pose group. Converts mm->m."""
    frames = []
    buf, buf_t = [], None
    for _, msg, t in bag.read_messages(topics=[topic]):
        pts = list(point_cloud2.read_points(msg, field_names=("x","y","z"),
                                             skip_nans=True))
        if not pts:
            continue
        pts = np.array(pts, dtype=np.float64) * 1e-3  # mm -> m
        r = np.linalg.norm(pts, axis=1)
        pts = pts[(r >= lo) & (r <= hi)]
        if len(pts) == 0:
            continue
        if buf_t is None:
            buf_t = t.to_sec()
        buf.append(pts)
        if len(buf) >= acc:
            frames.append((buf_t, np.vstack(buf)))
            buf, buf_t = [], None
    return frames


def read_camera_frames(bag, topic):
    frames = []
    for _, msg, t in bag.read_messages(topics=[topic]):
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        enc = msg.encoding
        if enc in ("bgr8", "rgb8"):
            img = arr.reshape(msg.height, msg.width, 3).copy()
            if enc == "rgb8":
                img = img[:, :, ::-1]
        elif enc == "mono8":
            img = cv2.cvtColor(arr.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
        elif enc == "bgra8":
            img = arr.reshape(msg.height, msg.width, 4)[:, :, :3].copy()
        else:
            continue
        frames.append((t.to_sec(), img))
    return frames


# ── Main ──────────────────────────────────────────────────────────────────────

def parse_args():
    p = argparse.ArgumentParser(
        description="VXS->Camera extrinsic calibration via checkerboard")
    p.add_argument("--bag",        required=True)
    p.add_argument("--cam",        required=True, help="ov9281_camera_info.yaml")
    p.add_argument("--cols",       type=int, default=9,  help="Interior corners X")
    p.add_argument("--rows",       type=int, default=8,  help="Interior corners Y")
    p.add_argument("--square",     type=float, default=0.05, help="Square size [m]")
    p.add_argument("--out",        default="calibration/T_vxs_cam.yaml")
    p.add_argument("--acc-frames", type=int,   default=5)
    p.add_argument("--min-range",  type=float, default=0.3)
    p.add_argument("--max-range",  type=float, default=2.0)
    p.add_argument("--pose-gap",   type=float, default=2.0,
                   help="Min seconds between poses to avoid duplicates")
    p.add_argument("--vxs-topic",  default="/vxs/pcloud/events")
    p.add_argument("--cam-topic",  default="/camera/image_raw")
    p.add_argument("--dump-dir",   default=None,
                   help="If set, save matched camera images + VXS PLY files here for inspection")
    p.add_argument("--fix-rotation", action="store_true",
                   help="Fix rotation to identity, only optimize translation (use when camera/VXS are parallel)")
    return p.parse_args()


def main():
    args = parse_args()

    print(f"Camera info : {args.cam}")
    K, D = load_camera_info(args.cam)
    print(f"  K = {K[0,0]:.1f} {K[1,1]:.1f}  cx={K[0,2]:.1f} cy={K[1,2]:.1f}")

    print(f"Checkerboard: {args.cols}x{args.rows}, square={args.square*100:.1f}cm")

    print(f"\nReading bag: {args.bag}")
    bag = rosbag.Bag(args.bag)
    topics = bag.get_type_and_topic_info().topics

    if args.vxs_topic not in topics:
        print(f"[ERROR] VXS topic '{args.vxs_topic}' not in bag. Available:")
        for t in topics: print(f"  {t}")
        sys.exit(1)

    print("  Reading VXS frames...")
    vxs_frames = read_vxs_frames(bag, args.vxs_topic, args.acc_frames,
                                  args.min_range, args.max_range)
    print(f"  {len(vxs_frames)} VXS frame groups ({args.acc_frames} frames each)")

    print("  Reading camera frames...")
    cam_frames = read_camera_frames(bag, args.cam_topic)
    print(f"  {len(cam_frames)} camera frames")
    bag.close()

    if not vxs_frames or not cam_frames:
        print("[ERROR] No frames found — check topics.")
        sys.exit(1)

    # ── Match and detect ──────────────────────────────────────────────────────
    cam_times = np.array([t for t, _ in cam_frames])
    vxs_planes, cam_planes = [], []
    last_pose_t = -999.0
    n_tried = 0

    dump_dir = None
    if args.dump_dir:
        dump_dir = Path(args.dump_dir)
        dump_dir.mkdir(parents=True, exist_ok=True)
        print(f"Debug dump dir: {dump_dir}")

    print(f"\nMatching frames and detecting checkerboard...")
    for vxs_t, vxs_pts in vxs_frames:
        if vxs_t - last_pose_t < args.pose_gap:
            continue

        idx = int(np.argmin(np.abs(cam_times - vxs_t)))
        dt = abs(cam_times[idx] - vxs_t)
        if dt > 1.0:
            continue

        n_tried += 1
        img = cam_frames[idx][1]

        result = detect_board(img, K, D, args.cols, args.rows, args.square)
        if result is None:
            print(f"  t={vxs_t:.1f}s — checkerboard NOT detected (dt={dt:.2f}s)")
            continue

        R_cb, t_cb = result
        n_cam, d_cam = board_plane_in_cam(R_cb, t_cb)
        dist_cam = float(np.linalg.norm(t_cb))  # camera-to-board distance

        plane = fit_plane_ransac(vxs_pts, dist_prior=dist_cam, dist_tol=0.15)
        if plane is None:
            print(f"  t={vxs_t:.1f}s — VXS plane fit FAILED ({len(vxs_pts)} pts, dist_prior={dist_cam:.2f}m)")
            continue

        n_vxs, d_vxs = plane

        # Sanity check: VXS plane distance should be within 25% of camera distance
        rejected = abs(d_vxs - dist_cam) > 0.25 * dist_cam
        pose_idx = len(vxs_planes) + 1

        # Dump debug files if requested
        if dump_dir is not None:
            tag = f"pose_{pose_idx:03d}_t{vxs_t:.1f}_{'REJECTED' if rejected else 'OK'}"
            # Save camera image with checkerboard drawn
            img_dbg = img.copy()
            gray_dbg = cv2.cvtColor(img_dbg, cv2.COLOR_BGR2GRAY)
            cb_flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            found_dbg, corners_dbg = cv2.findChessboardCorners(gray_dbg, (args.cols, args.rows), cb_flags)
            if found_dbg:
                cv2.drawChessboardCorners(img_dbg, (args.cols, args.rows), corners_dbg, found_dbg)
            cv2.putText(img_dbg, f"dist_cam={dist_cam:.3f}m d_vxs={d_vxs:.3f}m",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0) if not rejected else (0,0,255), 2)
            cv2.imwrite(str(dump_dir / f"{tag}.jpg"), img_dbg)
            # Save VXS points as PLY
            ply_path = dump_dir / f"{tag}.ply"
            with open(ply_path, "w") as pf:
                pf.write("ply\nformat ascii 1.0\n")
                pf.write(f"element vertex {len(vxs_pts)}\n")
                pf.write("property float x\nproperty float y\nproperty float z\nend_header\n")
                for p in vxs_pts:
                    pf.write(f"{p[0]:.6f} {p[1]:.6f} {p[2]:.6f}\n")

        if rejected:
            print(f"  t={vxs_t:.1f}s — REJECTED: d_vxs={d_vxs:.3f}m vs dist_cam={dist_cam:.3f}m (bad plane fit)")
            continue

        vxs_planes.append((n_vxs, d_vxs))
        cam_planes.append((n_cam, d_cam))
        last_pose_t = vxs_t
        print(f"  pose {len(vxs_planes):2d}: t={vxs_t:.1f}s  "
              f"dist_cam={dist_cam:.3f}m  d_vxs={d_vxs:.3f}m  "
              f"n_vxs={n_vxs.round(3)}  pts={len(vxs_pts)}")

    print(f"\n{len(vxs_planes)} valid poses (tried {n_tried})")

    if len(vxs_planes) < 3:
        print("[ERROR] Need at least 3 poses.")
        print("  Tips:")
        print("  - Hold board still for 3-5s per pose")
        print("  - Use good lighting for checkerboard detection")
        print("  - Try --acc-frames 3 to get more frame groups")
        print("  - Try --pose-gap 1.5 if poses are spaced close together")
        sys.exit(1)

    # ── Optimize ──────────────────────────────────────────────────────────────
    if args.fix_rotation:
        print("\nOptimizing T_VXS_cam (3 DOF — translation only, rotation fixed to identity)...")
        def cost_t_only(t, vxs_planes, cam_planes):
            R = np.eye(3)
            total = 0.0
            for (n_v, d_v), (n_c, d_c) in zip(vxs_planes, cam_planes):
                n_c_vxs = R @ n_c
                d_c_vxs = d_c - float(n_c_vxs @ t)
                cos_a = float(np.clip(n_v @ n_c_vxs, -1.0, 1.0))
                total += (1.0 - cos_a) * 500.0
                total += (d_v - d_c_vxs) ** 2 * 500.0
            return total
        res = minimize(cost_t_only, np.zeros(3), args=(vxs_planes, cam_planes),
                       method="Nelder-Mead",
                       options={"maxiter": 100000, "xatol": 1e-8, "fatol": 1e-8})
        # Wrap into full params
        full_params = np.array([0.0, 0.0, 0.0, res.x[0], res.x[1], res.x[2]])
        class _Res: pass
        best_res = _Res()
        best_res.success = res.success
        best_res.fun = res.fun
        best_res.x = full_params
    else:
        print("\nOptimizing T_VXS_cam (6 DOF)...")
        best_res = None
        starts = [(0,0,0)]
        for d in [0.05, -0.05, 0.1, -0.1]:
            for i in range(3):
                r = [0.0, 0.0, 0.0]; r[i] = d
                starts.append(tuple(r))
        for rx, ry, rz in starts:
            x0 = np.array([rx, ry, rz, 0.0, 0.0, 0.0])
            res = minimize(cost_fn, x0, args=(vxs_planes, cam_planes),
                           method="Nelder-Mead",
                           options={"maxiter": 200000, "xatol": 1e-8, "fatol": 1e-8})
            if best_res is None or res.fun < best_res.fun:
                best_res = res

    print(f"  Converged: {best_res.success}  cost: {best_res.fun:.6f}")

    T = T_from_params(best_res.x)
    R = T[:3, :3]
    t = T[:3, 3]
    rot = Rotation.from_matrix(R)

    print(f"\nT_VXS_cam (VXS <- camera):")
    print(np.round(T, 6))
    print(f"\nTranslation [m]:  {t.round(4)}")
    print(f"Rotation [deg]:   {np.degrees(rot.as_rotvec()).round(2)}")
    print(f"Rotation [euler]: {np.degrees(rot.as_euler('xyz')).round(2)} (XYZ)")

    # ── Per-pose residuals ────────────────────────────────────────────────────
    print("\nPer-pose residuals:")
    angle_errs, dist_errs = [], []
    for i, ((n_v, d_v), (n_c, d_c)) in enumerate(zip(vxs_planes, cam_planes)):
        n_c_vxs = R @ n_c
        d_c_vxs = d_c - float(n_c_vxs @ t)
        angle_err = float(np.degrees(np.arccos(np.clip(abs(n_v @ n_c_vxs), 0, 1))))
        dist_err = abs(d_v - d_c_vxs) * 1000
        angle_errs.append(angle_err)
        dist_errs.append(dist_err)
        print(f"  pose {i+1:2d}: angle={angle_err:.2f}deg  dist={dist_err:.1f}mm")

    print(f"\nMean angle error : {np.mean(angle_errs):.2f} deg")
    print(f"Mean dist  error : {np.mean(dist_errs):.1f} mm")

    # ── Save ──────────────────────────────────────────────────────────────────
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    out = {
        "T_vxs_cam": {
            "comment": "VXS LiDAR to camera transform (4x4, row-major)",
            "rows": 4, "cols": 4,
            "data": [round(v, 8) for v in T.flatten().tolist()]
        },
        "rotation_euler_xyz_deg": [round(v, 4) for v in
                                    np.degrees(rot.as_euler("xyz")).tolist()],
        "translation_m": [round(v, 6) for v in t.tolist()],
        "num_poses": len(vxs_planes),
        "mean_angle_error_deg": round(float(np.mean(angle_errs)), 4),
        "mean_dist_error_mm":   round(float(np.mean(dist_errs)), 4),
    }
    with open(args.out, "w") as f:
        yaml.dump(out, f, default_flow_style=False, sort_keys=False)
    print(f"\nSaved: {args.out}")


if __name__ == "__main__":
    main()

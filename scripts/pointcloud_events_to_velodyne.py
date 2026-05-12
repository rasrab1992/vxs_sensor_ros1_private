#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2


def range_filter(pts, min_range, max_range):
    """Remove points outside [min_range, max_range] metres from origin."""
    radii = np.linalg.norm(pts[:, :3], axis=1)
    mask = np.ones(len(pts), dtype=bool)
    if min_range > 0.0:
        mask &= radii >= min_range
    if max_range > 0.0:
        mask &= radii <= max_range
    return pts[mask]


def voxel_downsample(pts, voxel_size):
    """Keep one point per voxel (same logic as turntable voxel_downsample_numpy)."""
    if len(pts) == 0:
        return pts
    coords = pts[:, :3]
    vox_idx = np.floor(coords / voxel_size).astype(np.int32)
    keys = vox_idx[:, 0] * 1_000_003 + vox_idx[:, 1] * 1_009 + vox_idx[:, 2]
    order = np.argsort(keys)
    _, first_idx = np.unique(keys[order], return_index=True)
    return pts[order][first_idx]


def radius_outlier_removal(pts, radius, min_neighbors):
    """Remove points with fewer than min_neighbors within radius (same as turntable remove_radius_outlier)."""
    if len(pts) < min_neighbors + 1:
        return pts
    from scipy.spatial import cKDTree
    tree = cKDTree(pts[:, :3])
    counts = tree.query_ball_point(pts[:, :3], r=radius, return_length=True)
    # counts includes the point itself, so threshold is min_neighbors + 1
    return pts[counts > min_neighbors]


def statistical_outlier_removal(pts, k=10, std_ratio=2.0):
    """Remove points whose mean kNN distance > mean + std_ratio*std (same as turntable remove_statistical_outlier)."""
    if len(pts) < k + 1:
        return pts
    from scipy.spatial import cKDTree
    tree = cKDTree(pts[:, :3])
    dists, _ = tree.query(pts[:, :3], k=k + 1)
    mean_dist = dists[:, 1:].mean(axis=1)   # exclude self (col 0)
    threshold = mean_dist.mean() + std_ratio * mean_dist.std()
    return pts[mean_dist < threshold]


class EventsToVelodyne:
    def __init__(self):
        self.in_topic  = rospy.get_param("~in_topic",  "/vxs/pcloud/events")
        self.out_topic = rospy.get_param("~out_topic", "/vxs/pcloud/events_velo")
        self.default_intensity = float(rospy.get_param("~default_intensity", 1.0))
        self.default_ring      = int(rospy.get_param("~default_ring", 0))

        # Stage 1 — range filter: discard points outside [min_range, max_range] (set 0.0 to disable)
        self.min_range  = float(rospy.get_param("~min_range",  0.0))
        self.max_range  = float(rospy.get_param("~max_range",  2.0))
        # Stage 2 — voxel downsampling (set 0.0 to disable)
        self.voxel_size = float(rospy.get_param("~voxel_size", 0.02))
        # Stage 3 — radius outlier removal: point needs >= min_neighbors within radius (set 0 to disable)
        self.ror_radius = float(rospy.get_param("~ror_radius", 0.05))
        self.ror_min_nb = int(rospy.get_param("~ror_min_neighbors", 5))
        # Stage 4 — statistical outlier removal (set sor_k=0 to disable)
        self.sor_k      = int(rospy.get_param("~sor_k", 10))
        self.sor_std    = float(rospy.get_param("~sor_std_ratio", 2.0))

        self.pub = rospy.Publisher(self.out_topic, PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber(self.in_topic, PointCloud2, self.cb, queue_size=1)

        rospy.loginfo("[events_to_velodyne] min_range=%.2fm  max_range=%.1fm  voxel=%.3fm  "
                      "ROR r=%.3fm nb=%d  SOR k=%d std=%.1f",
                      self.min_range, self.max_range, self.voxel_size,
                      self.ror_radius, self.ror_min_nb, self.sor_k, self.sor_std)

    def cb(self, msg):
        # Read x,y,z,t from incoming VXS event cloud
        raw = list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z", "t"), skip_nans=True))
        if len(raw) == 0:
            return

        pts = np.array(raw, dtype=np.float64)   # (N, 4): x y z t

        # t is session-relative (seconds since first event of session).
        # Velodyne 'time' field must be per-frame offset in seconds (0 .. ~1/fps).
        # Subtract the earliest t in this frame so time[i] = offset from frame start.
        t_frame_start = pts[:, 3].min()

        # --- Stage 1: range filter ---
        if self.min_range > 0.0 or self.max_range > 0.0:
            pts = range_filter(pts, self.min_range, self.max_range)

        # --- Stage 2: voxel downsampling ---
        if self.voxel_size > 0.0 and len(pts) > 0:
            pts = voxel_downsample(pts, self.voxel_size)

        # --- Stage 3: radius outlier removal ---
        if self.ror_radius > 0.0 and self.ror_min_nb > 0 and len(pts) > self.ror_min_nb + 1:
            pts = radius_outlier_removal(pts, self.ror_radius, self.ror_min_nb)

        # --- Stage 4: statistical outlier removal ---
        if self.sor_k > 0 and len(pts) > self.sor_k + 1:
            pts = statistical_outlier_removal(pts, k=self.sor_k, std_ratio=self.sor_std)

        if len(pts) == 0:
            return

        # Build velodyne-format output: x,y,z,intensity,time,ring
        # time = per-point offset in seconds from frame start (required by LI-Init Velodyne parser)
        out_pts = [
            (float(p[0]), float(p[1]), float(p[2]),
             self.default_intensity, float(p[3]) - t_frame_start, self.default_ring)
            for p in pts
        ]

        fields = [
            PointField(name="x",         offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name="y",         offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name="z",         offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="time",      offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring",      offset=20, datatype=PointField.UINT16,  count=1),
        ]

        out_msg = point_cloud2.create_cloud(msg.header, fields, out_pts)
        out_msg.header.frame_id = msg.header.frame_id
        out_msg.header.stamp    = msg.header.stamp
        self.pub.publish(out_msg)


if __name__ == "__main__":
    rospy.init_node("pointcloud_events_to_velodyne")
    EventsToVelodyne()
    rospy.spin()

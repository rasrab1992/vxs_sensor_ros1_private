#!/usr/bin/env python3
import struct

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2


def rgb_to_float(r, g, b):
    rgb_uint32 = (int(r) << 16) | (int(g) << 8) | int(b)
    return struct.unpack("f", struct.pack("I", rgb_uint32))[0]


class XYZToXYZRGB:
    def __init__(self):
        self.in_topic = rospy.get_param("~in_topic", "/vxs/pcloud/cloud")
        self.out_topic = rospy.get_param("~out_topic", "/vxs/pcloud/cloud_rgb")
        self.rgb = rgb_to_float(255, 255, 255)

        self.pub = rospy.Publisher(self.out_topic, PointCloud2, queue_size=1)
        self.sub = rospy.Subscriber(self.in_topic, PointCloud2, self.cb, queue_size=1)

    def cb(self, msg):
        points = []
        for p in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            points.append((p[0], p[1], p[2], self.rgb))

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        out = point_cloud2.create_cloud(msg.header, fields, points)
        out.header.frame_id = msg.header.frame_id
        out.header.stamp = msg.header.stamp
        self.pub.publish(out)


if __name__ == "__main__":
    rospy.init_node("pointcloud_xyz_to_xyzrgb")
    XYZToXYZRGB()
    rospy.spin()


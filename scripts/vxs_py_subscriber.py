#!/usr/bin/env python3

import sys
import os
import ctypes
import numpy as np
from matplotlib import pyplot as plt

# Import ros stuff
import rospy

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

# OpenCV
import cv2
from cv_bridge import CvBridge, CvBridgeError


class VxsSensorSubscriber:

    def __init__(self):
        self.K = None

        # Calibration subscriber
        self.calibration_sub = rospy.Subscriber(
            "/sensor/camera_info", CameraInfo, self.CameraInfoCB
        )

        # Depth image subscriber
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/vxs_publisher/depth/image", Image, self.DepthCB
        )

        # Pointcloud subscriber
        self.pointcloud_sub = rospy.Subscriber(
            "/vxs_publisher/pcloud/cloud", PointCloud2, self.PointcloudCB
        )

        # Timestamped pointcloud (events as 3D XYZT points) subscriber
        self.pointcloud_sub = rospy.Subscriber(
            "/vxs_publisher/pcloud/events", PointCloud2, self.StampedPointcloudCB
        )

    def CameraInfoCB(self, cam_info_msg):
        if self.K is not None:
            return
        self.K = np.array(cam_info_msg.K, dtype=np.float32).reshape(3, 3)
        self.P = np.array(cam_info_msg.P, dtype=np.float(32)).reshape(3, 4)
        self.R = np.array(cam_info_msg.R, dtype=np.float(32)).reshape(3, 3)
        self.d = np.array(cam_info_msg.D, dtype=np.float(32))
        print("Calibration acquired!")

    def DepthCB(self, depth_img_msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(depth_img_msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return
        (rows, cols, channels) = cv_image.shape
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

    def PointcloudCB(self, pcloud_msg):
        # Here we convert the 'msg', which is of the type PointCloud2.
        # I ported the function read_points2 from
        # the ROS1 package.
        # https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py

        pcd_as_numpy_array = np.array(list(read_points(pcloud_msg)))

        for pt in pcd_as_numpy_array:
            print(pt)

    def StampedPointcloudCB(self, evcloud_msg):
        # Here we convert the 'msg', which is of the type PointCloud2.
        # Same as above, ported function 'read_points2' from the ROS1 package.
        # https://github.com/ros/common_msgs/blob/noetic-devel/sensor_msgs/src/sensor_msgs/point_cloud2.py

        pcd_as_numpy_array = np.array(list(read_points(evcloud_msg)))

        # NOTE: The 4th field will be a double that needs to be assigned (bitwise to a 64-bit int)
        for pt in pcd_as_numpy_array:
            x = pt[0]
            y = pt[1]
            z = pt[2]
            t = DoubleToInt64(pt[3])
            print(f"(x, y, z, t) = ({x}, {y}, {z}, {t})")


def DoubleToInt64(double_value):
    """
    Converts a double-precision floating-point number (double) to a 64-bit integer.

    Args:
      double_value: The double-precision floating-point number to convert.

    Returns:
      A 64-bit integer representing the bit pattern of the double.
    """
    return struct.unpack("<q", struct.pack("<d", double_value))[0]


## The code below is "ported" from
# https://github.com/ros/common_msgs/tree/noetic-devel/sensor_msgs/src/sensor_msgs
from collections import namedtuple
import ctypes
import math
import struct

_DATATYPES = {}
_DATATYPES[PointField.INT8] = ("b", 1)
_DATATYPES[PointField.UINT8] = ("B", 1)
_DATATYPES[PointField.INT16] = ("h", 2)
_DATATYPES[PointField.UINT16] = ("H", 2)
_DATATYPES[PointField.INT32] = ("i", 4)
_DATATYPES[PointField.UINT32] = ("I", 4)
_DATATYPES[PointField.FLOAT32] = ("f", 4)
_DATATYPES[PointField.FLOAT64] = ("d", 8)


def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.
    @param cloud: The point cloud to read from.
    @type  cloud: L{sensor_msgs.PointCloud2}
    @param field_names: The names of fields to read. If None, read all fields. [default: None]
    @type  field_names: iterable
    @param skip_nans: If True, then don't return any point with a NaN value.
    @type  skip_nans: bool [default: False]
    @param uvs: If specified, then only return the points at the given coordinates. [default: empty list]
    @type  uvs: iterable
    @return: Generator which yields a list of values for each point.
    @rtype:  generator
    """
    assert isinstance(cloud, PointCloud2), "cloud is not a sensor_msgs.msg.PointCloud2"
    fmt = _get_struct_fmt(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = (
        cloud.width,
        cloud.height,
        cloud.point_step,
        cloud.row_step,
        cloud.data,
        math.isnan,
    )
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in range(height):
                offset = row_step * v
                for u in range(width):
                    yield unpack_from(data, offset)
                    offset += point_step


def _get_struct_fmt(is_bigendian, fields, field_names=None):
    fmt = ">" if is_bigendian else "<"

    offset = 0
    for field in (
        f
        for f in sorted(fields, key=lambda f: f.offset)
        if field_names is None or f.name in field_names
    ):
        if offset < field.offset:
            fmt += "x" * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print(
                "Skipping unknown PointField datatype [%d]" % field.datatype,
                file=sys.stderr,
            )
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt


def main(args):
    print("INITIALIZING SUBSCRIBER NODE!")

    rospy.init_node("vxs_py_subscriber", anonymous=True)
    ic = VxsSensorSubscriber()

    rospy.spin()
    print("Shutting down... exiting")
    rospy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)

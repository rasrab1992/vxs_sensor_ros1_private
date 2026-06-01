#!/usr/bin/env python3
"""Quick camera viewer — subscribes to /camera/image_raw and shows with cv2."""
import sys
import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

bridge = CvBridge()

def cb(msg):
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    cv2.imshow("OV9281", img)
    cv2.waitKey(1)

rospy.init_node("view_camera", anonymous=True)
topic = rospy.get_param("~topic", "/camera/image_raw")
rospy.Subscriber(topic, Image, cb, queue_size=1, buff_size=2**24)
rospy.loginfo("Viewing %s  —  press Q in window to quit", topic)
rospy.spin()
cv2.destroyAllWindows()

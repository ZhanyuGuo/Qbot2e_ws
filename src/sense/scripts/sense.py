#!/usr/bin/env python
import rospy
import sys
import cv2
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo


class Sense:
    def __init__(self):
        self.info_sub = rospy.Subscriber(
            "/camera/rgb/camera_info", CameraInfo, self.info_cb, queue_size=1
        )
        self.rgb_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.rgb_cb, queue_size=1
        )
        self.dep_sub = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.dep_cb, queue_size=1
        )
        pass

    def info_cb(self, data):
        # get info only once
        self.info_sub.unregister()
        pass

    def rgb_cb(self, data):
        pass

    def dep_cb(self, data):
        pass

    pass


def main(args):
    rospy.init_node("sense")
    sense = Sense()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)

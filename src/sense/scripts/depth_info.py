#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


def image_cb(data):
    global bridge

    # cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # (rows, cols, channels) = cv_image.shape
    # data = Image()
    # print(data.data)
    # print(data.step, data.encoding)
    print(data.data)

    pass


def main():
    global bridge

    rospy.init_node("sense")

    bridge = CvBridge()
    rospy.Subscriber("/camera/depth/image_raw", Image, image_cb, queue_size=1)

    rospy.spin()
    pass


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
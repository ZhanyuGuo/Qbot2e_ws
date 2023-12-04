#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion


def motion_capture_cb(data):
    posistion = data.pose.position
    oriention = data.pose.orientation

    x = posistion.x
    y = posistion.y

    _, _, theta = euler_from_quaternion([oriention.x, oriention.y, oriention.z, oriention.w])

    info = "(x, y, theta) = ({:.2f}, {:.2f}, {:.2f})".format(x, y, theta)
    rospy.loginfo(info)


def main():
    rospy.init_node("motion_capture_listener")
    motion_capture_sub = rospy.Subscriber("/vrpn_client_node/Qbot2e_120/pose", PoseStamped, motion_capture_cb, queue_size=1)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main()

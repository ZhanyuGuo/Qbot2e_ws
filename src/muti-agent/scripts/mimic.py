#!/usr/bin/env python
import rospy
import sys

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

import numpy as np

class Mimic:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0
        
        self.x_d = 0
        self.y_d = 0
        self.theta_d = 0

        # gzy2 control parameters
        self.kk = 0.2
        self.l = 0.2

        self.my_odom_sub = rospy.Subscriber("/A/odom", Odometry, self.my_odom_cb, queue_size=1)
        self.other_odom_sub = rospy.Subscriber("/B/odom", Odometry, self.other_odom_cb, queue_size=1)

        self.vel_pub = rospy.Publisher(
            "/A/mobile_base/commands/velocity", Twist, queue_size=1
        )

        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        pass

    def gzy_stabilize_2(self):
        e_x = self.x_d - self.x
        e_y = self.y_d - self.y
        u_x = self.kk * e_x
        u_y = self.kk * e_y
        A = np.array(
            [
                [np.cos(self.theta), -self.l * np.sin(self.theta)],
                [np.sin(self.theta), self.l * np.cos(self.theta)],
            ]
        )
        U = np.array([[u_x], [u_y]])
        v_w = np.linalg.solve(A, U)
        v = v_w[0]
        w = v_w[1]
        return v, w
    
    def timer_cb(self, data):
        v, w = self.gzy_stabilize_2()

        vel = Twist()
        vel.linear.x = v
        vel.angular.z = w

        self.vel_pub.publish(vel)
        pass


    def my_odom_cb(self, data):
        posistion = data.pose.pose.position
        oriention = data.pose.pose.orientation

        self.x = posistion.x
        self.y = posistion.y

        _, _, self.theta = euler_from_quaternion(
            [oriention.x, oriention.y, oriention.z, oriention.w]
        )

        info = "(self.x, self.y, theta) = ({}, {}, {})".format(
            self.x, self.y, self.theta
        )
        rospy.loginfo(info)
        pass

    def other_odom_cb(self, data):
        posistion = data.pose.pose.position
        oriention = data.pose.pose.orientation

        self.x_d = posistion.x
        self.y_d = posistion.y + 1

        _, _, self.theta_d = euler_from_quaternion(
            [oriention.x, oriention.y, oriention.z, oriention.w]
        )

        info = "(self.x, self.y, theta) = ({}, {}, {})".format(
            self.x_d, self.y_d, self.theta_d
        )
        # rospy.loginfo(info)
        pass


    pass


def main(args):
    rospy.init_node("mimic")
    mimic = Mimic()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)
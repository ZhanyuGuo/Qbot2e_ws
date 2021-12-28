#!/usr/bin/env python
from matplotlib.pyplot import xcorr
import rospy
import sys

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

import numpy as np
from alogorithm.apf import apf


class Follower:
    def __init__(self):
        # position in global
        self.g_x = 0
        self.g_y = 2.24

        self.g_x_l = 1.12
        self.g_y_l = 1.12

        self.g_x_o = 0
        self.g_y_o = 0

        self.g_x_b = 2.24
        self.g_y_b = 1.12
        # end of positon in global

        self.x_o = 0
        self.y_o = 0
        self.theta_o = 0

        self.x_l = 0
        self.y_l = 0
        self.theta_l = 0

        self.x = 0
        self.y = 0
        self.theta = 0

        # for point choosing
        self.x_last = 0
        self.y_last = 0

        self.x_d = 0
        self.y_d = 0
        self.theta_d = 0

        # gzy2 control parameters
        self.kk = 0.5
        self.l = 0.175

        # self.x_b = 2.24
        # self.y_b = 1.12 - 2.24
        self.x_b = self.g_x_b - self.g_x
        self.y_b = self.g_y_b - self.g_y

        self.followerA_odom_sub = rospy.Subscriber(
            "/followerA/odom", Odometry, self.followerA_odom_cb, queue_size=1
        )
        self.followerB_odom_sub = rospy.Subscriber(
            "/followerB/odom", Odometry, self.followerB_odom_cb, queue_size=1
        )
        self.leader_odom_sub = rospy.Subscriber(
            "/leader/odom", Odometry, self.leader_odom_cb, queue_size=1
        )

        self.vel_pub = rospy.Publisher(
            "/followerA/mobile_base/commands/velocity", Twist, queue_size=1
        )

        rospy.Timer(rospy.Duration(0.2), self.timer_cb)
        pass

    def gzy_stabilization_2(self, x, y, theta, x_d, y_d):
        e_x = x_d - x
        e_y = y_d - y
        u_x = self.kk * e_x
        u_y = self.kk * e_y
        A = np.array(
            [
                [np.cos(theta), -self.l * np.sin(theta)],
                [np.sin(theta), self.l * np.cos(theta)],
            ]
        )
        U = np.array([[u_x], [u_y]])
        v_w = np.linalg.solve(A, U)
        v = v_w[0]
        w = v_w[1]

        return v, w

    def timer_cb(self, data):
        self.x_d, self.y_d = apf(
            self.x,
            self.y,
            self.x_l,
            self.y_l,
            [self.x_o, self.x_b],
            [self.y_o, self.y_b],
            60,
            self.x_last,
            self.y_last,
        )
        self.x_last = self.x
        self.y_last = self.y

        # v, w = self.gzy_stabilize_2()
        v, w = self.gzy_stabilization_2(self.x, self.y, self.theta, self.x_d, self.y_d)

        vel = Twist()
        vel.linear.x = v
        vel.angular.z = w

        self.vel_pub.publish(vel)
        pass

    def followerA_odom_cb(self, data):
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

    def followerB_odom_cb(self, data):
        posistion = data.pose.pose.position
        oriention = data.pose.pose.orientation

        self.x_o = posistion.x + self.g_x_o - self.g_x
        self.y_o = posistion.y + self.g_y_o - self.g_y

        _, _, self.theta_o = euler_from_quaternion(
            [oriention.x, oriention.y, oriention.z, oriention.w]
        )

        # info = "(self.x, self.y, theta) = ({}, {}, {})".format(
        #     self.x_o, self.y_o, self.theta_o
        # )
        # rospy.loginfo(info)
        pass

    def leader_odom_cb(self, data):
        posistion = data.pose.pose.position
        oriention = data.pose.pose.orientation

        self.x_l = posistion.x + self.g_x_l - self.g_x
        self.y_l = posistion.y + self.g_y_l - self.g_y

        _, _, self.theta_l = euler_from_quaternion(
            [oriention.x, oriention.y, oriention.z, oriention.w]
        )

        # info = "(self.x, self.y, theta) = ({}, {}, {})".format(
        #     self.x_l, self.y_l, self.theta_d
        # )
        # rospy.loginfo(info)
        pass

    pass


def main(args):
    rospy.init_node("followerA")
    follower = Follower()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)

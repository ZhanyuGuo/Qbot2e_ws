#!/usr/bin/env python
import rospy
import sys
import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

import numpy as np


class Stabilization:
    def __init__(self, x_d=1.0, y_d=1.0, controller="lpj"):
        controllers = ["lpj", "guyue", "gzy", "gzy2"]
        assert controller in controllers, "Controller not defined."
        self.controller = controller

        # lpj control parameters
        self.kp = 0.15
        self.ka = 0.2
        self.kb = -0.15

        # guyue control parameters
        self.kv = 0.1
        self.kw = 0.7

        # gzy control parameters
        self.k1 = 0.2
        self.k2 = 1.0
        self.k3 = 0.2
        self.theta_d = 0
        self.dmax = math.sqrt(x_d ** 2 + x_d ** 2)

        # gzy2 control parameters
        self.kk = 0.2
        self.l = 0.2

        self.x_d = x_d
        self.y_d = y_d

        self.x = 0
        self.y = 0
        self.theta = 0

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)

        self.vel_pub = rospy.Publisher(
            "/mobile_base/commands/velocity", Twist, queue_size=1
        )

        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

    def lpj_stabilize(self):
        ruo = math.sqrt((self.x_d - self.x) ** 2 + (self.y_d - self.y) ** 2)
        beta = -math.atan2(self.y_d - self.y, self.x_d - self.x)
        alpha = -self.theta - beta

        v = self.kp * ruo
        w = self.ka * alpha + self.kb * beta

        info = "ruo = {}, alpha = {}, beta = {}".format(ruo, alpha, beta)
        # rospy.loginfo(info)

        return v, w

    def gzy_stabilize(self):
        e_x = self.x_d - self.x
        e_y = self.y_d - self.y
        d = math.sqrt(e_x ** 2 + e_y ** 2)

        beta = math.atan2(self.y_d - self.y, self.x_d - self.x)
        alpha = beta - self.theta

        v = self.k1 * d
        lda = d / self.dmax
        w = lda * self.k2 * alpha + (1 - lda) * self.k3 * (self.theta_d - self.theta)
        return v, w

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

    def guyue_stabilize(self):
        if self.theta < 0:
            self.theta = self.theta + 2 * math.pi

        d_e = math.sqrt(
            math.pow((self.x_d - self.x), 2) + math.pow((self.y_d - self.y), 2)
        )

        if (self.y_d - self.y) == 0 and (self.x_d - self.x) > 0:
            theta_d = 0
        if (self.y_d - self.y) > 0 and (self.x_d - self.x) > 0:
            theta_d = math.atan((self.y_d - self.y) / (self.x_d - self.x))
        if (self.y_d - self.y) > 0 and (self.x_d - self.x) == 0:
            theta_d = 0.5 * math.pi
        if (self.y_d - self.y) > 0 and (self.x_d - self.x) < 0:
            theta_d = math.atan((self.y_d - self.y) / (self.x_d - self.x)) + math.pi
        if (self.y_d - self.y) == 0 and (self.x_d - self.x) < 0:
            theta_d = math.pi
        if (self.y_d - self.y) < 0 and (self.x_d - self.x) < 0:
            theta_d = math.atan((self.y_d - self.y) / (self.x_d - self.x)) + math.pi
        if (self.y_d - self.y) < 0 and (self.x_d - self.x) == 0:
            theta_d = 1.5 * math.pi
        if (self.y_d - self.y) < 0 and (self.x_d - self.x) > 0:
            theta_d = math.atan((self.y_d - self.y) / (self.x_d - self.x)) + 2 * math.pi

        theta_e = theta_d - self.theta
        if theta_e < -math.pi:
            theta_e = theta_e + 2 * math.pi
        if theta_e > math.pi:
            theta_e = theta_e - 2 * math.pi

        v = self.kv * d_e
        w = self.kw * theta_e
        return v, w

    def timer_cb(self, data):
        if self.controller == "lpj":
            v, w = self.lpj_stabilize()
        elif self.controller == "guyue":
            v, w = self.guyue_stabilize()
        elif self.controller == "gzy":
            v, w = self.gzy_stabilize()
        elif self.controller == "gzy2":
            v, w = self.gzy_stabilize_2()

        vel = Twist()
        vel.linear.x = v
        vel.angular.z = w

        self.vel_pub.publish(vel)

    def odom_cb(self, data):
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


def main(args):
    rospy.init_node("stabilization")
    stabilization = Stabilization(0.0, -1.0, "lpj")
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)

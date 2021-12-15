#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from tf.transformations import euler_from_quaternion

from math import sqrt
from math import atan2
from math import pi
import math
import numpy as np
import matplotlib.pyplot as plt

k = 0
k1 = 0.3
k2 = 0.5
k3 = 0.7

x = 0
y = 0
theta = 0


def genTraj():
    t = np.linspace(0, 60, num=600)
    w = 0.5*np.ones(t.shape)
    r = 1.0
    v = w*r
    x, y = r*np.cos(w*t), r*np.sin(w*t)
    theta = math.pi/2 + w*t

    return x, y, theta, v, w


def lpj_trajectory(x, y, theta):
    global x_d, y_d, theta_d, v_d, w_d, k

    e_x = x_d[k] - x
    e_y = y_d[k] - y
    v_r = v_d[k]
    w_r = w_d[k]

    x_e = e_x*math.cos(theta) + e_y*math.sin(theta)
    y_e = -e_x*math.sin(theta) + e_y*math.cos(theta)

    theta_e = theta_d[k] - theta

    v = v_r*math.cos(theta_e) + k2*x_e
    w = w_r + k1*v_r*y_e + k3*math.sin(theta_e)

    k += 1
    return v, w


def odom_cb(data):
    global x, y, theta

    posistion = data.pose.pose.position
    oriention = data.pose.pose.orientation

    x = posistion.x
    y = posistion.y

    _, _, theta = euler_from_quaternion(
        [oriention.x, oriention.y, oriention.z, oriention.w])

    info = "(x, y, theta) = ({}, {}, {})".format(x, y, theta)
    rospy.loginfo(info)


def main():
    global x_d, y_d, theta_d, v_d, w_d, x, y, theta, k

    rospy.init_node("trajectory")

    rospy.Subscriber('/odom', Odometry, odom_cb, queue_size=1)
    velocityPublisher = rospy.Publisher(
        '/mobile_base/commands/velocity', Twist, queue_size=1)

    rate = rospy.Rate(10.0)

    x_d, y_d, theta_d, v_d, w_d = genTraj()
    xList = []
    yList = []

    while k < 600:
        v, w = lpj_trajectory(x, y, theta)
        xList.append(x)
        yList.append(y)

        vel = Twist()
        vel.linear.x = v
        vel.angular.z = w
        velocityPublisher.publish(vel)
        rate.sleep()

    plt.scatter(xList, yList, c='b')
    plt.scatter(x_d, y_d, c='r')
    plt.show()

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

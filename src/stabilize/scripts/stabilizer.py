#!/usr/bin/env python
import rospy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from tf.transformations import euler_from_quaternion

from math import sqrt
from math import atan2
from math import pi

epi = 0.02
x_d = 1.0
y_d = 1.0
theta_d = 0
Kp, Ka, Kb = 0.15, 0.2, -0.15

x = 0
y = 0
theta = 0


def lpj_control(x, y, theta):
    # if sqrt((x_d - x)**2 + (y_d - y)**2) < epi:
    #     v = 0
    #     w = 0.5 * (theta_d - theta)
    #     pass
    # else:

    ruo = sqrt((x_d - x)**2 + (y_d - y)**2)
    beta = -atan2(y_d - y, x_d - x)
    alpha = -theta - beta
    v = Kp*ruo
    w = Ka*alpha + Kb*beta
    info = "ruo = {}, alpha = {}, beta = {}".format(ruo, alpha, beta)

    rospy.loginfo(info)
    
    # It is no need to control the angle at the destination.

    # if v < epi:
    #     w = 0.5 * (theta_d - theta)

    return v, w


def odom_cb(data):
    global x, y, theta

    posistion = data.pose.pose.position
    oriention = data.pose.pose.orientation

    x = posistion.x
    y = posistion.y

    _, _, theta = euler_from_quaternion([oriention.x, oriention.y, oriention.z, oriention.w])

    info = "(x, y, theta) = ({}, {}, {})".format(x, y, theta)
    rospy.loginfo(info)


def main():
    rospy.init_node("stabilizer")

    rospy.Subscriber('/odom', Odometry, odom_cb, queue_size=1)
    velocityPublisher = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        v, w = lpj_control(x, y, theta)
        vel = Twist()
        vel.linear.x = v
        vel.angular.z = w
        velocityPublisher.publish(vel)
        rate.sleep()

    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

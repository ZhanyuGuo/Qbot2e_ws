#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

x = 0
y = 0
x_o = 0
y_o = 0
z_o = 0
w_o = 1
yaw_t = 0
liner_speed = 0
angular_speed = 0

xTarget = 1
yTarget = 1


def odom_cb(msg):
    global x
    global y
    global x_o
    global y_o
    global z_o
    global w_o
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    w_o = msg.pose.pose.orientation.w
    x_o = msg.pose.pose.orientation.x
    y_o = msg.pose.pose.orientation.y
    z_o = msg.pose.pose.orientation.z


if __name__ == '__main__':
    rospy.init_node('stabilzer_2')

    velocityPublisher = rospy.Publisher(
        '/mobile_base/commands/velocity', Twist, queue_size=1)
    rospy.Subscriber('/odom', Odometry, odom_cb, queue_size=1)

    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        vel = Twist()
        (_, _, yaw) = euler_from_quaternion([x_o, y_o, z_o, w_o])

        if yaw < 0:
            yaw = yaw + 2 * math.pi

        D_err = math.sqrt(math.pow((xTarget - x), 2) +
                          math.pow((xTarget - y), 2))

        if (yTarget - y) == 0 and (xTarget - x) > 0:
            yaw_t = 0
        if (yTarget - y) > 0 and (xTarget - x) > 0:
            yaw_t = math.atan((yTarget - y) / (xTarget - x))
        if (yTarget - y) > 0 and (xTarget - x) == 0:
            yaw_t = 0.5 * math.pi
        if (yTarget - y) > 0 and (xTarget - x) < 0:
            yaw_t = math.atan((yTarget - y) / (xTarget - x)) + math.pi
        if (yTarget - y) == 0 and (xTarget - x) < 0:
            yaw_t = math.pi
        if (yTarget - y) < 0 and (xTarget - x) < 0:
            yaw_t = math.atan((yTarget - y) / (xTarget - x)) + math.pi
        if (yTarget - y) < 0 and (xTarget - x) == 0:
            yaw_t = 1.5 * math.pi
        if (yTarget - y) < 0 and (xTarget - x) > 0:
            yaw_t = math.atan((yTarget - y) / (xTarget - x)) + 2 * math.pi

        Theta_err = yaw_t - yaw
        if Theta_err < -math.pi:
            Theta_err = Theta_err + 2 * math.pi
        if Theta_err > math.pi:
            Theta_err = Theta_err - 2 * math.pi

        liner_speed = 0.1 * D_err
        angular_speed = 0.7 * Theta_err

        vel.linear.x = liner_speed
        vel.angular.z = angular_speed

        velocityPublisher.publish(vel)

        rate.sleep()

    rospy.spin()

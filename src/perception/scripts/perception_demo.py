#!/usr/bin/env python
import rospy
import sys
import cv2
import math
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose2D

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

import colorRecognition
import QRcodeRecognition


class Perception:
    def __init__(self, controller="lpj"):
        controllers = ["lpj", "gzy2"]
        assert controller in controllers, "Controller not defined."
        self.controller = controller

        self.bridge = CvBridge()
        self.rgb_image = None
        self.dep_image = None
        self.K = None

        self.info_sub = rospy.Subscriber(
            "/camera/rgb/camera_info", CameraInfo, self.info_cb, queue_size=1
        )
        self.rgb_sub = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.rgb_cb, queue_size=1
        )
        self.dep_sub = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.dep_cb, queue_size=1
        )

        self.x = 0
        self.y = 0
        self.theta = 0

        # lpj control parameters
        self.kp = 0.1
        self.ka = 0.3
        self.kb = -0.15

        # gzy2 control parameters
        self.kk = 0.1
        self.l = 0.2

        self.vel_old = Twist()

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        self.vel_pub = rospy.Publisher(
            "/mobile_base/commands/velocity", Twist, queue_size=1
        )

        rospy.Timer(rospy.Duration(0.1), self.timer_cb)
        pass

    def info_cb(self, data):
        # get info only once
        self.info_sub.unregister()

        self.K = data.K
        self.K = np.array(self.K)
        self.K = np.reshape(self.K, (3, 3))

        print(self.K)
        pass

    def rgb_cb(self, data):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # print("rgb", self.rgb_image.shape)
        except CvBridgeError as e:
            print(e)

        # cv2.imshow("Image window", self.rgb_image)
        # cv2.waitKey(3)
        pass

    def dep_cb(self, data):
        try:
            self.dep_image = self.bridge.imgmsg_to_cv2(data)
            # print("dep", self.dep_image.shape)
        except CvBridgeError as e:
            print(e)
        pass

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
        # rospy.loginfo(info)

    def timer_cb(self, data):
        try:
            frame = self.rgb_image

            # u, v = colorRecognition.color_recog(frame)
            u, v = QRcodeRecognition.qrcode_recog(frame)

            cv2.imshow("Image window", frame)
            cv2.waitKey(3)

            if u != 0 or v != 0:
                d = self.dep_image[int(v)][int(u)] / 1000.0
                x, y, _ = self.getCoordinateInWorld(u, v, d)

                self.x_d, self.y_d = x, y
                print(
                    "target = ({}, {}), current = ({}, {}).".format(
                        self.x_d, self.y_d, self.x, self.y
                    )
                )

                if self.controller == "lpj":
                    v, w = self.lpj_stabilization(
                        self.x, self.y, self.theta, self.x_d, self.y_d
                    )
                elif self.controller == "gzy2":
                    v, w = self.gzy_stabilization_2(
                        self.x, self.y, self.theta, self.x_d, self.y_d
                    )

                vel = Twist()
                vel.linear.x = v
                vel.angular.z = w
                self.vel_pub.publish(vel)

                self.vel_old.linear.x = v
                self.vel_old.angular.z = w
            else:
                self.vel_pub.publish(self.vel_old)
        except Exception as e:
            print(e)
        pass

    def getCoordinateInWorld(self, u, v, z):
        x_c = z * (u - self.K[0, 2]) / self.K[0, 0]
        y_c = z * (v - self.K[1, 2]) / self.K[1, 1]
        z_c = z

        x_c, y_c, z_c = z_c, -x_c, -y_c

        x_w = x_c * math.cos(self.theta) - y_c * math.sin(self.theta) + self.x
        y_w = x_c * math.sin(self.theta) + y_c * math.cos(self.theta) + self.y
        z_w = z

        return x_w, y_w, z_w

    def lpj_stabilization(self, x, y, theta, x_d, y_d):
        ruo = math.sqrt((x_d - x) ** 2 + (y_d - y) ** 2)
        beta = -math.atan2(y_d - y, x_d - x)
        alpha = -theta - beta

        v = self.kp * ruo
        w = self.ka * alpha + self.kb * beta

        # info = "ruo = {}, alpha = {}, beta = {}".format(ruo, alpha, beta)
        # rospy.loginfo(info)

        return v, w

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

    pass


def main(args):
    rospy.init_node("perception_demo")

    perception = Perception("gzy2")

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)

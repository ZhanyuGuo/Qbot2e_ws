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


class Sense:
    def __init__(self):
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

        # self.pose_pub = rospy.Publisher("/target", Pose2D)




        self.x = 0
        self.y = 0
        self.theta = 0

        self.kp = 0.15
        self.ka = 0.2
        self.kb = -0.15

        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size=1)
        self.vel_pub = rospy.Publisher(
            "/mobile_base/commands/velocity", Twist, queue_size=1
        )

        rospy.Timer(rospy.Duration(0.1), self.timer_cb)

        pass

    def info_cb(self, data):
        # get info only once
        self.info_sub.unregister()
        # TODO
        self.K = data.K
        self.K = np.array(self.K)
        self.K = np.reshape(self.K, (3, 3))
        print(self.K)
        print(type(self.K))

        pass

    def lpj_stabilize(self):
        ruo = math.sqrt((self.x_d - self.x) ** 2 + (self.y_d - self.y) ** 2)
        beta = -math.atan2(self.y_d - self.y, self.x_d - self.x)
        alpha = -self.theta - beta

        v = self.kp * ruo
        w = self.ka * alpha + self.kb * beta

        info = "ruo = {}, alpha = {}, beta = {}".format(ruo, alpha, beta)
        # rospy.loginfo(info)

        return v, w

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
    
    def rgb_cb(self, data):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            # print("rgb", self.rgb_image.shape)
            pass
        except CvBridgeError as e:
            print(e)
            pass

        # cv2.imshow("Image window", self.rgb_image)
        # cv2.waitKey(3)

        pass

    def dep_cb(self, data):
        try:
            self.dep_image = self.bridge.imgmsg_to_cv2(data)
            # print("dep", self.dep_image.shape)
            pass
        except CvBridgeError as e:
            print(e)
            pass
        pass

    def timer_cb(self, data):
        try:
            frame = self.rgb_image

            u, v = colorRecognition.color_recog(frame)

            # cv2.imshow("Image window", frame)
            # cv2.waitKey(3)

            # print(u, v)
            # u, v = 320, 240
            if u == 0 and v == 0:
                x = self.x + 0.2
                y = self.y
                pass
            else:
                z = self.dep_image[v, u]
                # print(z)
                x, y, _ = self.getCoordinateInCamera(u, v, z/1000.0)
                
            # print(x, y)

            self.x_d, self.y_d = x, y

            v, w = self.lpj_stabilize()
            print(v, w)

            vel = Twist()
            vel.linear.x = v
            vel.angular.z = w

            self.vel_pub.publish(vel)

            print("Publish success.")
            # target = Pose2D()
            # target.x = x
            # target.y = y
            # target.theta = 0
            # self.pose_pub.publish(target)
        except:
            pass
        pass

    def getCoordinateInCamera(self, u, v, z):
        x = z*(u - self.K[0, 2]) / self.K[0, 0]
        y = z*(v - self.K[1, 2]) / self.K[1, 1]
        z = z

        x, y, z = z, -x, -y
        return x, y, z

    pass


def main(args):
    rospy.init_node("sense")
    sense = Sense()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main(sys.argv)

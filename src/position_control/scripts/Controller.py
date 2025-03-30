#!/usr/bin/env python
import os

import numpy as np
import pandas as pd
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion


class Controller:
    def __init__(self, robot_num=0):
        self.robot_num = robot_num
        self.robot_name = f"qbot_12{robot_num}"
        self.x, self.y, self.th = 0, 0, 0
        self.k, self.l = 0.5, 0.2

        # load desired position from xlsx file
        file_dir = os.path.dirname(__file__)
        xlsx_path = os.path.join(file_dir, f"qbot_position.xlsx")
        df = pd.read_excel(xlsx_path, sheet_name=self.robot_name)
        self.pos_des_list = df.values.tolist()

        self.has_odom, self.has_start = False, False
        self.odom_sub = rospy.Subscriber(f"{self.robot_name}/odom", Odometry, self.odomCallback, queue_size=1)
        self.start_sub = rospy.Subscriber("/start", Bool, self.startCallback, queue_size=1)
        self.twist_pub = rospy.Publisher(f"{self.robot_name}/mobile_base/commands/velocity", Twist, queue_size=1)

        self.timer = rospy.Timer(rospy.Duration(0.01), self.timerCallback)
        self.cnt = 0

    def odomCallback(self, msg: Odometry):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.x, self.y = position.x, position.y
        _, _, self.th = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        self.has_odom = True

    def startCallback(self, msg: Bool):
        if self.has_start:
            return

        if msg.data:
            self.has_start = True

    def timerCallback(self, e):
        if not self.has_odom or not self.has_start:
            return

        if self.cnt >= len(self.pos_des_list):
            return

        pos_des = self.pos_des_list[self.cnt]
        v, w = self.controller(pos_des[0], pos_des[1])
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        self.twist_pub.publish(twist)
        self.cnt += 1

    def controller(self, x_d, y_d):
        x, y, th = self.x, self.y, self.th
        k, l = self.k, self.l

        e_x, e_y = x_d - x, y_d - y
        u_x, u_y = k * e_x, k * e_y
        A = np.array([[np.cos(th), -l * np.sin(th)], [np.sin(th), l * np.cos(th)]])
        U = np.array([[u_x], [u_y]])

        vw = np.linalg.solve(A, U)
        v, w = vw[0], vw[1]
        return v, w

    def spin(self):
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            print("shutting down")


if __name__ == "__main__":
    controller = Controller(0)

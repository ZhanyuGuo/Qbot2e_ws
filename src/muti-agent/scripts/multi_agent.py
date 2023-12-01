#!/usr/bin/env python
import rospy
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
import numpy as np
import matplotlib.pyplot as plt
import csv


class Controller:
    def __init__(self):
        self.odom_sub_120 = rospy.Subscriber("/Qbot2e_120/odom", Odometry, self.odom_cb_120, queue_size=1)
        self.odom_sub_121 = rospy.Subscriber("/Qbot2e_121/odom", Odometry, self.odom_cb_121, queue_size=1)
        self.odom_sub_122 = rospy.Subscriber("/Qbot2e_122/odom", Odometry, self.odom_cb_122, queue_size=1)
        self.vel_pub_120 = rospy.Publisher("/Qbot2e_120/mobile_base/commands/velocity", Twist, queue_size=1)
        self.vel_pub_121 = rospy.Publisher("/Qbot2e_121/mobile_base/commands/velocity", Twist, queue_size=1)
        self.vel_pub_122 = rospy.Publisher("/Qbot2e_122/mobile_base/commands/velocity", Twist, queue_size=1)

        self.T = 0.1
        self.target_v = 0.25
        self.real_v_120 = 0
        self.real_v_121 = 0
        self.real_v_122 = 0

        self.gain_120 = np.array([-0.8731, -0.5208, -0.0447])
        self.gain_121 = np.array([-0.6831, -0.5135, 0.1694])
        self.gain_122 = np.array([-0.7018, -0.2820, 0.0818])

        self.count = 0
        self.err_list_120 = []
        self.err_list_121 = []
        self.err_list_122 = []
        self.time_list = []
        self.count_plot = 0

        rospy.Timer(rospy.Duration(self.T), self.timer_cb)

    def odom_cb_120(self, data):
        linear_v = data.twist.twist.linear
        self.real_v_120 = linear_v.x

    def odom_cb_121(self, data):
        linear_v = data.twist.twist.linear
        self.real_v_121 = linear_v.x

    def odom_cb_122(self, data):
        linear_v = data.twist.twist.linear
        self.real_v_122 = linear_v.x

    def timer_cb(self, data):
        if self.count * self.T <= 3:
            self.next_v_121 = 0
            self.next_v_120 = 0.1
            self.next_v_122 = 0.15
        else:
            if self.count_plot * self.T <= 25:
                bias = 0.007
                bias1 = 0.01
                self.err_list_122.append(self.real_v_122 - self.target_v + bias)
                self.err_list_120.append(self.real_v_120 - self.target_v + bias)
                self.err_list_121.append(self.real_v_121 - self.real_v_120 + bias1)

                self.time_list.append(self.count_plot * self.T)
                self.count_plot += 1
            else:
                with open("/home/zames/Desktop/data.txt", "w") as file:
                    writer = csv.writer(file)
                    writer.writerow(self.time_list)
                    writer.writerow(self.err_list_122)
                    writer.writerow(self.err_list_120)
                    writer.writerow(self.err_list_121)

                plt.plot(self.time_list, self.err_list_122, label="Node 1", color="yellow")
                plt.plot(self.time_list, self.err_list_120, label="Node 2", color="blue")
                plt.plot(self.time_list, self.err_list_121, label="Node 3", color="red")

                plt.axis([0, 25, -0.20, 0.02])
                plt.xlabel("Time(s)")
                plt.ylabel("Velocity error")
                plt.legend()
                plt.show()
                sys.exit()

            if self.count % 10 == 0:
                self.pre_v_120 = self.real_v_120
                error_v_120 = self.real_v_120 - self.target_v
                self.command_buf_120 = self.gain_120.dot(error_v_120) * 1
                command_acce_120 = self.command_buf_120[0]
                self.next_v_120 = self.pre_v_120 + command_acce_120

                self.pre_v_121 = self.real_v_121
                error_v_121 = self.real_v_121 - self.real_v_120
                self.command_buf_121 = self.gain_121.dot(error_v_121) * 1
                command_acce_121 = self.command_buf_121[0]
                self.next_v_121 = self.pre_v_121 + command_acce_121

                self.pre_v_122 = self.real_v_122
                error_v_122 = self.real_v_122 - self.target_v
                self.command_buf_122 = self.gain_122.dot(error_v_122) * 1
                command_acce_122 = self.command_buf_122[0]
                self.next_v_122 = self.pre_v_122 + command_acce_122

        vel_120 = Twist()
        vel_120.linear.x = self.next_v_120
        vel_120.angular.z = 0.6
        self.vel_pub_120.publish(vel_120)

        vel_121 = Twist()
        vel_121.linear.x = self.next_v_121
        vel_121.angular.z = 0.6
        self.vel_pub_121.publish(vel_121)

        vel_122 = Twist()
        vel_122.linear.x = self.next_v_122
        vel_122.angular.z = 0.6
        self.vel_pub_122.publish(vel_122)

        self.count += 1
        real_v_info_120 = "(vx_120)=({})".format(self.real_v_120)
        rospy.loginfo(real_v_info_120)
        real_v_info_121 = "(vx_121)=({})".format(self.real_v_121)
        rospy.loginfo(real_v_info_121)
        real_v_info_122 = "(vx_122)=({})".format(self.real_v_122)
        rospy.loginfo(real_v_info_122)
        print(self.count * self.T)


def main(args):
    rospy.init_node("controller")
    controller = Controller()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")


if __name__ == "__main__":
    main(sys.argv)

#!/usr/bin/env python
import rospy
from Controller import Controller

if __name__ == "__main__":
    robot_num = 0
    rospy.init_node(f"qbot_12{robot_num}")
    controller = Controller(robot_num)
    controller.spin()

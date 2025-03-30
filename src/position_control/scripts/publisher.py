#!/usr/bin/env python
import rospy
import pandas as pd
from geometry_msgs.msg import Twist

robot_name = "qbot_120"
sheet_num = 0
rospy.init_node(f"publisher_{robot_name}")

twist_pub = rospy.Publisher(f"{robot_name}/mobile_base/commands/velocity", Twist, queue_size=1)

cnt = 0

xlsx_path = "/home/zhang/qbot_vw_final_dt001.xlsx"
df = pd.read_excel(xlsx_path, sheet_name=f"vehicle{sheet_num}")
vel_list = df.values.tolist()

twist_list = []
for vel in vel_list:
    twist = Twist()
    twist.linear.x = vel[0]
    twist.angular.z = vel[1]
    twist_list.append(twist)

def timer_cb(e):
    global cnt
    print(cnt)

    twist_pub.publish(twist_list[cnt])
    cnt += 1

rospy.Timer(rospy.Duration(0.01), timer_cb)
try:
    rospy.spin()
except KeyboardInterrupt:
    print("Shutting down")
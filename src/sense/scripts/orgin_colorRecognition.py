# -*- coding: utf-8 -*-
"""
# －－－－湖南创乐博智能科技有限公司－－－－
#  文件名：40_colorDetection.py
#  版本：V2.0
#  author: zhulin778
#  说明：颜色检测
#####################################################
"""
import cv2
import numpy as np

cap = cv2.VideoCapture(1)

# 设定yellow的阈值
lower_yellow = np.array([20, 120, 150])
upper_yellow = np.array([55, 180, 255])
# 设定red的阈值
lower_red = np.array([0, 200, 80])
upper_red = np.array([20, 255, 255])
# 设定white的阈值
lower_white = np.array([40, 0, 140])
upper_white = np.array([90, 30, 255])
# 设定orange的阈值
lower_orange = np.array([0, 150, 170])
upper_orange = np.array([25, 230, 255])
# 设定green的阈值 robust
lower_green = np.array([55, 120, 100])
upper_green = np.array([80, 200, 255])
# 设定blue的阈值 robust
lower_blue = np.array([80, 100, 80])
upper_blue = np.array([150, 255, 255])
while 1:
    # 获取每一帧
    ret, frame = cap.read()
    # 转换到HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    print(hsv[200, 200, :])
    cv2.circle(frame, (200, 200), 10, color=(0, 0, 255))
    # 根据阈值构建掩模
    mask = cv2.inRange(hsv, lower_blue, upper_blue)

    # 膨胀腐蚀
    mask = cv2.dilate(mask, None, iterations=10)
    mask = cv2.erode(mask, None, iterations=10)

    # 对mask检测轮廓
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    # print('cnt:', cnts)
    center = None

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        ((x, y), radius) = cv2.minEnclosingCircle(c)

        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            print("radius:", radius)
            print("center:", x, y)
    # 对原图像和掩模进行位运算
    # res=cv2.bitwise_and(frame,frame,mask=mask)

    # 显示图像
    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)
    #  cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
# 关闭窗口
cv2.destroyAllWindows()

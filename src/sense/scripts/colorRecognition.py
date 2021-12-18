# -*- coding: utf-8 -*-


# from os import WCOREDUMP
from typing import OrderedDict
import cv2
import numpy as np


# ---------------------------location---------------------------------------
# pixel : length = 166 : 100    (px:mm)
# roboArm origin:(40,40)
px_len_ratio = 166 / 100
robotOrigin = np.array([40, 40])

# ---------------------------color---------------------------------------
# 设定yellow的阈值
lower_yellow = np.array([20, 120, 150])
upper_yellow = np.array([55, 180, 255])
# 设定red的阈值
lower_red = np.array([0, 200, 80])
upper_red = np.array([20, 255, 255])
# 设定white的阈值
# lower_white = np.array([40, 0, 140])
# upper_white = np.array([90, 30, 255])
# 设定orange的阈值
lower_orange = np.array([0, 150, 170])
upper_orange = np.array([25, 230, 255])
# 设定green的阈值 robust
lower_green = np.array([55, 120, 100])
upper_green = np.array([80, 200, 255])
# 设定blue的阈值 robust
lower_blue = np.array([80, 100, 80])
upper_blue = np.array([150, 255, 255])


colorList = ["red", "yellow", "blue", "green", "orange"]
lower_thresh = [lower_red, lower_yellow, lower_blue, lower_green, lower_orange]
upper_thresh = [upper_red, upper_yellow, upper_blue, upper_green, upper_orange]


def judge_color(hsv):
    colorID = -1
    # 根据阈值构建掩模
    for i in range(len(colorList)):
        mask = cv2.inRange(hsv, lower_thresh[i], upper_thresh[i])

        # 膨胀腐蚀
        mask = cv2.dilate(mask, None, iterations=10)
        mask = cv2.erode(mask, None, iterations=10)

        # 对mask检测轮廓
        cnts = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )[-2]

        if len(cnts) > 0:
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)

            if radius > 20:
                colorID = i
                break
                # cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # print('radius:', radius)
                # print('center:', x, y)
                # return colorID

    print(i, colorList[colorID])
    return colorID


def get_world_loc(cam_loc):
    world_loc = (np.array(cam_loc) - robotOrigin) / px_len_ratio
    return world_loc


def color_recog(frame):
    # 转换到HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    cv2.circle(frame, (40, 40), 10, color=(0, 0, 255))

    # 根据阈值构建掩模
    colorID = judge_color(hsv)
    mask = cv2.inRange(hsv, lower_thresh[colorID], upper_thresh[colorID])
    # mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 膨胀腐蚀
    mask = cv2.dilate(mask, None, iterations=10)
    mask = cv2.erode(mask, None, iterations=10)

    # 对mask检测轮廓
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]

    if len(cnts) > 0:
        c = max(cnts, key=cv2.contourArea)
        (center, radius) = cv2.minEnclosingCircle(c)
        x, y = center
        if radius > 10:
            cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
            print("radius:", radius)
            print("center:", center)
            world_loc = get_world_loc([x, y])
            print("world_loc:", world_loc)

        return colorID, center
    else:
        return 0, (0, 0)


if __name__ == "__main__":

    cap = cv2.VideoCapture(0)

    while 1:
        # 获取每一帧
        ret, frame = cap.read()

        color, center = color_recog(frame)
        print("color:", color)
        print("center:", center)

        # 显示图像
        cv2.imshow("frame", frame)
        # cv2.imshow('mask', mask)
        #  cv2.imshow('res',res)
        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    # 关闭窗口
    cv2.destroyAllWindows()

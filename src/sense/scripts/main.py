import numpy as np

# import os
import time
import cv2


# import myserial
import motion_planning
import robotArm_kenetics
import colorRecognition


if __name__ == "__main__":
    cap = cv2.VideoCapture(1)
    ret, frame = cap.read()
    if not ret:
        raise Exception("打开摄像头失败！\n")

    # ----------------------------读取图像和颜色判断-------------------------------------------------
    while 1:
        # 获取每一帧
        ret, frame = cap.read()

        color, center = colorRecognition.color_recog(frame)
        print("color:", color)
        print("center:", center)

        # 显示图像
        cv2.imshow("frame", frame)
        # cv2.imshow('mask', mask)
        #  cv2.imshow('res',res)

        # ----------------------------求解世界坐标-------------------------------------------------
        if color and center:
            world_loc = colorRecognition.get_world_loc(cam_loc=center)
            print("world_loc:", world_loc)
            moving_theta = robotArm_kenetics.get_moving_angel(
                robotArm_kenetics.start_location, end_pos=(*world_loc, 40)
            )
            print("moving_theta:", moving_theta)

        k = cv2.waitKey(5) & 0xFF
        if k == 27:
            break

    # 关闭窗口
    cv2.destroyAllWindows()

    # 关闭窗口
    cv2.destroyAllWindows()

# -*- coding=utf-8 -*-
import cv2 as cv
from pyzbar import pyzbar


def decodeDisplay(gray, image):
    barcodes = pyzbar.decode(gray)
    u, v = 0, 0
    for barcode in barcodes:
        (x, y, w, h) = barcode.rect
        cv.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
        u, v = x + w / 2, y + h / 2
        # print("center loc:", u, v)

        # 提取二维码数据为字节对象，所以如果我们想在输出图像上
        # 画出来，就需要先将它转换成字符串
        barcodeData = barcode.data.decode("UTF8")
        barcodeType = barcode.type

        # # 绘出图像上条形码的数据和条形码类型
        text = "{} ({})".format(barcodeData, barcodeType)
        cv.putText(
            image, text, (x, y - 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 125), 2
        )
        # # 向终端打印条形码数据和条形码类型
        # print("[INFO] Found {} barcode: {}".format(barcodeType, barcodeData))
    return u, v


def qrcode_recog(frame):
    gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    u, v = decodeDisplay(gray, frame)
    return u, v

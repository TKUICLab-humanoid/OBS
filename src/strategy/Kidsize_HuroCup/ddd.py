#!/usr/bin/env python
#coding=utf-8
import rospy
import numpy as np
from hello1 import Sendmessage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tku_msgs.msg import camera
import cv2 
import sys
import time

class deep_calculate:
    def __init__(self):
        self.bridge = CvBridge()
        self.Image_compress_sub = rospy.Subscriber("colormodel_image",Image, self.convert)			# 訂閱攝像頭資訊 #"/kidsize/camera/image_raw" #"compress_image" #"/usb_cam/image_raw"
        #self.Deep_Matrix = []
        # rospy.spin()

    def convert(self, imgmsg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(imgmsg, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv_image = cv2.resize(cv_image, (320, 240))
        cv_image_2 = cv2.resize(cv_image, (32, 24))

        self.Deep_Matrix = []
        for compress_width in range(0, 32, 1):
            self.Deep_Matrix.append(0)
            for compress_height in range(23, -1, -1):
                blue = cv_image_2.item(compress_height, compress_width, 0)
                green = cv_image_2.item(compress_height, compress_width, 1)
                red = cv_image_2.item(compress_height, compress_width, 2)
                if (blue == 128 and green == 0 and red == 128) or (blue == 128 and green == 128 and red == 0) :
                    self.Deep_Matrix[compress_width] = 23 - compress_height
                    break
                if compress_height == 0:
                    self.Deep_Matrix[compress_width] = 24
        #print(self.Deep_Matrix)
        # cv2.imshow("Image_show",cv_image)
        cv2.waitKey(1)

        #================================================================
        # hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        # lowera = np.array([78,43,46])
        # uppera = np.array([99,255,255])
        # mask = cv2.inRange(hsv,lowera,uppera)
        # edges = cv2.Canny(mask,50,150,apertureSize=5)

        # lines = cv2.HoughLinesP(edges,1,np.pi / 180,50,lines=None,minLineLength=0,maxLineGap=30)
        # # lines = cv2.HoughLines(edges,1,np.pi / 180,0)
        # # print("lines=",lines)
        # for line in lines:
        #     x1,y1,x2,y2 = line[0]
        #     cv2.line(cv_image,(x1,y1),(x2,y2),(255,255,255),5)

        #     x1 = float(x1)
        #     x2 = float(x2)
        #     y1 = float(y1)
        #     y2 = float(y2)

        #     print ('x1=',x1)
        #     print ('x2=',x2)
        #     print ('y1=',y1)
        #     print ('y2=',y2)

        #     if x1==0 or x2==0 or y1==0 or y2==0 :
        #         result=0
        #     elif x2 - x1 == 0:    #直线是竖直的
        #         result=90
        #     elif y2 - y1 == 0 :     #直线是水平的
        #         result=0
        #     else:
        #         # 计算斜率
        #         k = -(y2 - y1) / (x2 - x1)
        #         # 求反正切，再将得到的弧度转换为度
        #         result = np.arctan(k) * 57.29577
        # print('slope = ',float(result))
        
        # cv2.imshow("Image_show",cv_image)
        # # cv2.imshow("Image_show",edges)
        # cv2.waitKey(1)
        #===================================================================
if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            send = Sendmessage()
            if send.Web == True:
                pass
            if send.Web == False:
                deep_calculate()
                rospy.spin()
    except rospy.ROSInterruptException:
        pass

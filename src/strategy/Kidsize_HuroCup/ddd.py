#!/usr/bin/env python
#coding=utf-8
from re import T
import rospy
import numpy as np
from hello1 import Sendmessage
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tku_msgs.msg import camera
import cv2 
import sys
import time
import math

class deep_calculate:
    def __init__(self):
        self.bridge = CvBridge()
        self.Image_compress_sub = rospy.Subscriber("colormodel_image",Image, self.convert)			# 訂閱攝像頭資訊 #"/kidsize/camera/image_raw" #"compress_image" #"/usb_cam/image_raw"
        #self.Deep_Matrix = []
        # rospy.spin()
        self.first_red = True
        self.ya = 0
        self.aa = 0
        self.x1 = 0
        self.y1 = 0
        self.x2 = 1
        self.y2 = 0
        self.cnt = 0
        self.a = True
        self.b = True
        self.slope = 0
        self.degree = 0
        self.red_width = 0
        self.Y_Dy = 24

    def convert(self, imgmsg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(imgmsg, "bgr8")
        except CvBridgeError as e:
            print(e)
        cv_image = cv2.resize(cv_image, (320, 240))
        cv_image_2 = cv2.resize(cv_image, (32, 24))
        # cv_image_3 = cv2.resize(cv_image, (32, 24))
        # send = Sendmessage()
        # #註解掉ddd有值 python_image有sent error 沒註解掉python_image可跑值為0 ddd值為0

        self.red_width = 0
        self.R_Deep_Matrix = []
        for compress_width in range(0, 32, 1):
            self.r = True
            self.R_Deep_Matrix.append(0)
            for compress_height in range(23, -1, -1):
                blue = cv_image_2.item(compress_height, compress_width, 0)
                green = cv_image_2.item(compress_height, compress_width, 1)
                red = cv_image_2.item(compress_height, compress_width, 2)
                if (blue == 255 and green == 255 and red == 0) and (self.r == True) :
                    self.red_width += 1
                    self.r = False
                if (blue == 255 and green == 255 and red == 0):
                    self.R_Deep_Matrix[compress_width] = 23 - compress_height
                    break
                if compress_height == 0:
                    self.R_Deep_Matrix[compress_width] = 24
        # print('red_width = ',self.red_width)

        self.x1 = 0
        self.y1 = 0
        self.x2 = 1
        self.y2 = 0
        self.cnt = 0
        self.Y_L_Deep = 0
        self.Y_R_Deep = 0
        self.Y_C_Deep = 0
        self.Y_Deep_sum = 0
        self.a0 = 0
        self.a1 = 0
        self.a2 = 0
        self.a3 = 0
        self.a4 = 0
        self.line_flag = False
        self.Xa = 0
        self.Ya = 0
        self.Xmin = 0
        self.Ymin = 0
        flag = True

        for compress_width in range(0, 32, 1):
            self.a = True
            for compress_height in range(23, -1, -1):
                blue = cv_image_2.item(compress_height, compress_width, 0)
                green = cv_image_2.item(compress_height, compress_width, 1)
                red = cv_image_2.item(compress_height, compress_width, 2)

                if compress_height == 6:            #計算黃色像素格
                    if (blue == 128 and green == 128 and red == 0):
                        self.a0+=1
                elif compress_height == 7:
                    if (blue == 128 and green == 128 and red == 0):
                        self.a1+=1
                elif compress_height == 8:
                    if (blue == 128 and green == 128 and red == 0):
                        self.a2+=1
                elif compress_height == 9:
                    if (blue == 128 and green == 128 and red == 0):
                        self.a3+=1
                elif compress_height == 10:
                    if (blue == 128 and green == 128 and red == 0):
                        self.a4+=1

                if compress_width == 0 and compress_height == 0:        #計算左邊有無黃色
                    blue1 = blue
                    green1 = green
                    red1 = red
                if compress_width == 31 and compress_height == 0:       #計算右邊有無黃色
                    blue2 = blue
                    green2 = green
                    red2 = red

                if (blue == 255 and green == 255 and red == 0) :
                    if self.a == True :
                        self.Xa = compress_width
                        self.Ya = 23 - compress_height
                        self.cnt += 1
                        self.a = False
                if (blue == 255 and green == 255 and red == 0) and self.first_red == True:
                    self.first_red = False
                    self.x1 = compress_width
                    self.y1 = 23 - compress_height
                if abs(self.red_width - self.cnt) <= 2 and self.b == True:
                    self.b = False
                    self.x2 = compress_width
                    self.y2 = 23 - compress_height 
                if self.Ya == min(self.R_Deep_Matrix) :
                    if self.y1 > self.y2 :
                        if flag == True:
                            self.Xmin = self.Xa
                            self.Ymin = self.Ya
                            flag = False
                    elif self.y2 >= self.y1 :
                        self.Xmin = self.Xa
                        self.Ymin = self.Ya
        
        # print('a0 = ',self.a0)
        # print('----------')
        # print('a1 = ',self.a1)
        # print('----------')
        # print('a2 = ',self.a2)
        # print('----------')
        # print('a3 = ',self.a3)
        # print('----------')
        # print('a4 = ',self.a4)
        # print('----------')

        if (self.x2 - self.Xmin) == 0 or (self.Xmin - self.x1) == 0:
            if abs(self.Xmin - self.x1) <= abs(self.Xmin - self.x2):
                self.slope =  (self.y2 - self.Ymin) / 0.00001
            elif abs(self.Xmin - self.x1) > abs(self.Xmin - self.x2):
                self.slope =  (self.Ymin - self.y1) / 0.00001
        elif abs(self.Xmin - self.x1) <= abs(self.Xmin - self.x2):
            self.slope =  (self.y2 - self.Ymin) / (self.x2 - self.Xmin)
        elif abs(self.Xmin - self.x1) > abs(self.Xmin - self.x2):
            self.slope =  (self.Ymin - self.y1) / (self.Xmin - self.x1)
        self.degree = int(math.degrees(self.slope))
        self.first_red = True
        self.b = True
    #舊的斜率
        # if (self.x2 - self.x1) == 0:
        #     self.first_red = True
        #     self.b = True
        #     self.degree = 0
        #     self.slope = 0
        # else:
        #     self.first_red = True
        #     self.b = True
        #     self.slope =  (self.y2 - self.y1) / (self.x2 - self.x1) 
        #     self.degree = int(math.degrees(self.slope))
        
        # if (blue1 != 128 or green1 != 128 or red1 != 0) and (blue2 != 128 or green2 != 128 or red2 != 0) and (0 < (self.a0 + self.a1 + self.a2 + self.a3 + self.a4) <= 15):
        #     self.line_flag  = True#開flag

    #舊的斜率

        # print('Flag = ',line_flag)

        # print('=======================================')
        # print('x1 = ',self.x1)
        # print('y1 = ',self.y1)
        # print('self.cnt = ',self.cnt)
        # print('red_width = ',self.red_width)
        # print('x2 = ',self.x2)
        # print('y2 = ',self.y2)
        # print('slope = ',self.slope)
        # print('degree = ',self.degree)

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
        # print(self.Deep_Matrix)



        self.aa = self.Deep_Matrix
        # cv2.imshow("Image_show",cv_image)
        cv2.waitKey(1)


        self.Y_Deep_Matrix = []
        for compress_width in range(0, 32, 1):
            self.Y_Deep_Matrix.append(0)
            for compress_height in range(23, -1, -1):
                blue = cv_image_2.item(compress_height, compress_width, 0)
                green = cv_image_2.item(compress_height, compress_width, 1)
                red = cv_image_2.item(compress_height, compress_width, 2)
                if (blue == 128 and green == 128 and red == 0) :
                    self.Y_Deep_Matrix[compress_width] = 23 - compress_height
                    break
                if compress_height == 0:
                    self.Y_Deep_Matrix[compress_width] = 24
        # print(self.Y_Deep_Matrix)
        # print(send.color_mask_subject_size[1][0])
        self.ya = self.Y_Deep_Matrix
        # for j in range (0, 32, 1):
        # # Filter_Matrix.append(0)
        # # Filter_Matrix[j] = Focus_Matrix[j] - deep.ya[j]
        #     if self.ya[j] < self.Y_Dy:
        #         self.Y_Dy = self.ya[j]
        #     self.Y_Deep_sum += self.aa[j]
        #     self.Y_L_Deep = self.ya[4]
        #     self.Y_R_Deep = self.ya[28]
        #     self.Y_C_Deep = self.ya[16]
        # cv2.imshow("Image_show",cv_image)
        # print("Y_L_Deep = ",self.Y_L_Deep)
        # print("Y_C_Deep = ",self.Y_C_Deep)
        # print("Y_R_Deep = ",self.Y_R_Deep)

        cv2.waitKey(1)

        self.B_Deep_Matrix = []
        for compress_width in range(0, 32, 1):
            self.B_Deep_Matrix.append(0)
            for compress_height in range(23, -1, -1):
                blue = cv_image_2.item(compress_height, compress_width, 0)
                green = cv_image_2.item(compress_height, compress_width, 1)
                red = cv_image_2.item(compress_height, compress_width, 2)
                if (blue == 128 and green == 0 and red == 128):
                    self.B_Deep_Matrix[compress_width] = 23 - compress_height
                    break
                if compress_height == 0:
                    self.B_Deep_Matrix[compress_width] = 24
        # print(self.Deep_Matrix)
        self.ba = self.B_Deep_Matrix
        # cv2.imshow("Image_show",cv_image)
        cv2.waitKey(1)
        #================================================================
        # hsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        # lowera = np.array([78,43,46])
        # uppera = np.array([99,255,255])
        # mask = cv2.inRange(hsv,lowera,uppera)
        # edges = cv2.Canny(mask,50,150)

        # lines = cv2.HoughLinesP(edges,1,np.pi / 180,100,lines=None,minLineLength=0,maxLineGap=30)
        # # lines = cv2.HoughLines(edges,1,np.pi / 180,0)
        # # print("lines=",lines)
        # for line in lines:
        #     x1,y1,x2,y2 = line[0]
        #     cv2.line(cv_image,(x1,y1),(x2,y2),(255,255,255),5)

        #     x1 = float(x1)
        #     x2 = float(x2)
        #     y1 = float(y1)
        #     y2 = float(y2)

        #     # print ('x1=',x1)
        #     # print ('x2=',x2)
        #     # print ('y1=',y1)
        #     # print ('y2=',y2)

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
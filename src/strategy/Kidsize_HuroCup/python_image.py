#!/usr/bin/env python
#coding=utf-8
# from turtle import st
import rospy
import numpy as np
from hello1 import Sendmessage
from ddd import deep_calculate
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tku_msgs.msg import camera
import cv2 
import sys
import time

Filter_Matrix = []
Xc = 0
Dy = 24
WR = 0
WL = 0
Xb = 0
Dx = 0
Xc_count = 0
Xc_num = 0
Now_speed = 0
Goal_speed = 0
Turn_angle_status = 0
Straight_status = 0
Turn_status = 0
Angle = 0
max_speed_flag = False
walking = False

#==============================image===============================
def Image_Init():
    global Filter_Matrix, Xc, Dy, WR, WL, Xb, Dx, Xc_count, Xc_num
    Filter_Matrix = []
    Xc = 0
    Dy = 24
    WR = 0
    WL = 0
    Xb = 0
    Dx = 0
    Xc_count = 0
    Xc_num = 0



def Image_Info():
    print()
    print('==============================================================')
    print()
    # print(Focus_Matrix)
    # print(deep.Deep_Matrix)
    print(Filter_Matrix)
    # print('WR = '+ str(WR))
    # print('WL = '+ str(WL))
    # print('Xb = '+ str(Xb))
    # print('Xc_count = '+ str(Xc_count))
    # print('Xc_num = '+ str(Xc_num))
    # print('Xc = '+ str(Xc))
    print('Dx = '+ str(Dx))
    print('Dy = '+ str(Dy))

def Normal_Obs_Parameter():
    global Filter_Matrix, Xc, Dy, WR, WL, Xb, Dx, Xc_count, Xc_num
    for i in range (0, 32, 1):
        Filter_Matrix.append(0)
        Filter_Matrix[i] = Focus_Matrix[i] - deep.Deep_Matrix[i]
        if Filter_Matrix[i] >= 0 :
            Xc_count += 1
            Xc_num += i
            Xc = int(Xc_num) // int(Xc_count)
            pass
        else :
            Filter_Matrix[i] = 0
        WR += (32-i) * Filter_Matrix[i]
        WL += (i+1) * Filter_Matrix[i]
        if deep.Deep_Matrix[i] < Dy:
            Dy = deep.Deep_Matrix[i]
    if WL > WR:
        Xb = 31
    elif WL <= WR:
        Xb = 0
    Dx = Xc - Xb
#=============================strategy=============================
def Straight_Speed():
    global Goal_speed 
    if Dy ==24 or max_speed_flag == True:
        Goal_speed = 1500
        pass
    elif 16 <= Dy < 24:
        Goal_speed = 1000
        pass
    elif 8 <= Dy < 16:
        Goal_speed = 500
        pass
    elif 0 <= Dy < 8:
        Goal_speed = 0
        pass
    print( 'Goal_speed = ' + str(Goal_speed))

def Turn_Angle(Turn_angle_status):
    global Angle 
    if Turn_angle_status == 0: #R
        print('turn right')
        Angle = -10
        pass
    elif Turn_angle_status == 1: #L
        print('turn left')
        Angle = 10
        pass
    else: 
        Angle = 0
    print( 'Angle = ' + str(Angle))

def Move(Straight_status=0,x=0,y=0,z=0,theta=0,sensor=0):
    print('Straight_status = ' + str(Straight_status))
    if Straight_status == 1:     #speed ++
        print('go')
        send.sendContinuousValue(x + Goal_speed,y,z,theta,sensor)
        pass
    elif Straight_status == 2:   #speed --
        send.sendContinuousValue(x + 2000,y,z,theta,sensor)
        pass
    elif Straight_status == 0:
        print('turn')
        send.sendContinuousValue(x,y,z,theta,sensor)
        send.sendContinuousValue(x + Goal_speed,y,z,theta + Angle,sensor)
        pass
    print( 'aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa = ' + str(theta + Angle))
# def Turn(Turn_status=0,x=0,y=0,z=0,theta=0,sensor=0):
#     print('Turn_status = ' + str(Turn_status))
#     if Turn_status == 1:     #L
#         print('turn left')
#         send.sendContinuousValue(x,y,z,theta + 5,sensor)
#         pass
#     elif Turn_status == 0: #R
#         print('turn right')
#         send.sendContinuousValue(x,y,z,theta - 5,sensor)
#     pass

# def IMU_Fix():
#     pass
# def Turn_Head():
#     pass

if __name__ == '__main__':
    try:
        print("try main")
        deep = deep_calculate()
        send = Sendmessage()
        while not rospy.is_shutdown():
            # if send.Web == True:
            #     print("send.Web = True")
            #     pass
            # print("walking")
            if send.is_start == True:
            # if send.Web == False:
                print("start")
                #==============================image===============================
                # Focus_Matrix = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
                Focus_Matrix = [7, 7, 7, 7, 9, 9, 9, 9, 11, 11, 11, 11, 12, 12, 13, 13, 13, 13, 12, 12, 11, 11, 11, 11, 9, 9, 9, 9, 7, 7, 7, 7]
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                #=============================strategy=============================
                send.sendHeadMotor(1,2048,100)
                send.sendHeadMotor(2,1520,100)
                if walking == False:
                    send.sendBodyAuto(0,0,0,0,1,0)
                    time.sleep(1.5) 
                walking = True
                
                max_speed_flag == False

                if Dy < 24:
                    if 17 > Dx > 2 :        #turn right
                        print('avoid')
                        Straight_Speed()
                        Turn_Angle(Turn_angle_status = 0)
                        Move(Straight_status = 0)
                        # Turn(Turn_status = 0)
                    elif -2 > Dx > -17:     #turn left
                        print('avoid')
                        Straight_Speed()
                        Turn_Angle(Turn_angle_status = 1)
                        Move(Straight_status = 0)
                        # Turn(Turn_status = 1)
                    elif 2 >= Dx >= -2:
                        print('no avoid')
                        max_speed_flag == True
                        # Straight_Speed()
                        Move(Straight_status = 2)
                        pass
                    pass
                elif Dy == 24:
                    print('go straight')
                    Straight_Speed()
                    Move(Straight_status = 1)
                    pass
            if send.is_start == False:
                print("stop")
                # print(send.is_start)
                if walking == True:
                    send.sendContinuousValue(0,0,0,0,0)
                    time.sleep(1.5) 
                    send.sendBodyAuto(0,0,0,0,1,0)
                    walking = False
                time.sleep(1)
                send.sendBodySector(29)
            print('walking ====== ' + str(walking))
    except rospy.ROSInterruptException:
        pass
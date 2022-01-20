#!/usr/bin/env python
#coding=utf-8
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
status = 0

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
    # print(Filter_Matrix)
    print('WR = '+ str(WR))
    print('WL = '+ str(WL))
    print('Xb = '+ str(Xb))
    # print('Xc_count = '+ str(Xc_count))
    # print('Xc_num = '+ str(Xc_num))
    print('Xc = '+ str(Xc))
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
'''def Straight_Speed(Dy):
    global Goal_speed
    if Dy ==24:
        Goal_speed = 3000
        pass
    elif 16 <= Dy < 24:
        Goal_speed = 2000
        pass
    elif 8 <= Dy < 16:
        Goal_speed = 1000
        pass
    elif 0 <= Dy < 8:
        Goal_speed = 500
        pass
def Straight_Move(Goal_speed,x=0,y=0,z=0,theta=0,sensor=0):
    global status
    if status == 1:     #speed ++
        while x < Goal_speed:
            x += 100
            send.sendContinuousValue(x,y,z,theta,sensor)
            pass
        pass
    elif status == 2:   #speed --
        while x > Goal_speed:
            x -= 100
            send.sendContinuousValue(x,y,z,theta,sensor)
            pass
        pass
    elif status == 0:
        while x == Goal_speed:
            send.sendContinuousValue(x,y,z,theta,sensor)
            pass
        pass'''
# def Turn(Dx):
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
            if send.Web == True:
                print("send.Web = True")
                pass
            if send.Web == False:
                #print("send.Web = False")
                #==============================image===============================
                Focus_Matrix = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                #=============================strategy=============================
                '''if Dy < 24:
                    if  > Goal_speed:
                        #speed--
                        Straight_Speed(Dy)
                        print('Goal_speed = ' + str(Goal_speed) )
                        Straight_Move(status = 2)
                        pass
                    elif Now_speed == Goal_speed:
                        #stand
                        Straight_Speed(Dy)
                        print('Goal_speed = ' + str(Goal_speed) )
                        Straight_Move(status = 0)
                        pass
                    pass
                elif Dy == 24:
                    #speed++ until maxspeed
                    Straight_Speed(Dy)
                    print('Goal_speed = ' + str(Goal_speed) )
                    Straight_Move(status = 1)
                    pass'''
                    #=====================================
                '''if Dy < 24:
                    if 17 > Dx > 2:
                        if Now_speed > Goal_speed:
                            print('go & speed -- R')
                            # go & speed -- 
                            # def Straight_Speed
                            # def Straight_Move(Goal_speed)
                            #Turn_Right
                            print('Turn_Right 1')
                            # def Turn(Dx)
                            pass
                        elif Now_speed == Goal_speed:
                            print('go R')
                            # go 
                            # def Straight_Speed
                            # def Straight_Move(Goal_speed)
                            #Turn_Right
                            print('Turn_Right 2')
                            # def Turn(Dx)
                            pass
                        pass
                    elif -2 > Dx > -17:
                        if Now_speed > Goal_speed:
                            print('go & speed -- L')
                            # go & speed -- 
                            # def Straight_Speed
                            # def Straight_Move(Goal_speed)
                            #Turn_Right
                            print('Turn_Left 1')
                            # def Turn(Dx)
                            pass
                        elif Now_speed == Goal_speed:
                            print('go L')
                            # go 
                            # def Straight_Speed
                            # def Straight_Move(Goal_speed)
                            #Turn_Right
                            print('Turn_Left 2')
                            # def Turn(Dx)
                            pass
                        pass
                    elif 2 >= Dx >= -2:
                        if Now_speed < Goal_speed:
                            print('walk straight after obs ++')
                            # go & speed ++ 
                            # def Straight_Speed
                            # def Straight_Move(Goal_speed)
                            pass
                        elif Now_speed == Goal_speed:
                            print('walk straight after obs')
                            # go 
                            # def Straight_Speed
                            # def Straight_Move(Goal_speed)
                            pass
                        pass
                    pass
                elif Dy == 24:
                    #go with max speed
                    print('walk straight with max speed')
                    pass'''
    except rospy.ROSInterruptException:
        pass
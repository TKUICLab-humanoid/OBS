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
walking = False
Yaw_wen = 0
imu_angle = 0
IMU_ok = False
Deep_sum = 0
R_deep_sum = 0
L_deep_sum = 0
L_Deep = 0
R_Deep = 0
red_flag = False
R_min = 0
R_max = 0
B_left = 0
B_right = 0
XMax_one = 0
XMin_one = 0
XMin_two = 0
XMax_two = 0
B_min = 0
B_max = 0
First_Reddoor = False
redoor_dis = False


#==============================image===============================
def Image_Init():
    global Filter_Matrix, Xc, Dy, WR, WL, Xb, Dx, Xc_count, Xc_num, Deep_sum, L_Deep, R_Deep, R_min, R_max, B_min, B_max, B_left, B_right, XMax_one, XMin_one, XMin_two, XMax_two
    Filter_Matrix = []
    Xc = 0
    Dy = 24
    WR = 0
    WL = 0
    Xb = 0
    Dx = 0
    Xc_count = 0
    Xc_num = 0
    Deep_sum = 0
    L_Deep = 0
    R_Deep = 0
    R_min = 0
    R_max = 0
    B_left = 0
    B_right = 0
    XMax_one = 0
    XMin_one = 0
    XMin_two = 0
    XMax_two = 0
    B_min = 0
    B_max = 0

def Image_Info():
    print()
    print('==============================================================')
    print()
    if red_flag == False:
        # print(Focus_Matrix)
        # print(deep.Deep_Matrix)
        # print(Filter_Matrix)
        # print('WR = '+ str(WR))
        # print('WL = '+ str(WL))
        # print('Xb = '+ str(Xb))
        # print('Xc_count = '+ str(Xc_count))
        # print('Xc_num = '+ str(Xc_num))
        # print('Xc = '+ str(Xc))
        print('Dx = '+ str(Dx))
        print('Dy = '+ str(Dy))
    else :
        print('red_flag = '+ str(red_flag))
        print('R_min = '+ str(R_min))
        print('R_max = '+ str(R_max))
        print('B_min = '+ str(B_min))
        print('B_max = '+ str(B_max))
        print('B_left = '+ str(B_left))
        print('B_right = '+ str(B_right))

def Normal_Obs_Parameter():
    global Filter_Matrix, Xc, Dy, WR, WL, Xb, Dx, Xc_count, Xc_num, Deep_sum, R_Deep, L_Deep, red_flag, R_min, R_max, B_min, B_max, B_left, B_right, XMax_one, XMin_one, XMin_two, XMax_two
    if send.color_mask_subject_size[5][0] > 1000 :
        red_flag = True
        R_min = send.color_mask_subject_XMin[5][0] 
        R_max = send.color_mask_subject_XMax[5][0]

        if send.color_mask_subject_cnts[2] == 1 :
            B_min = send.color_mask_subject_XMin[2][0]
            B_max = send.color_mask_subject_XMax[2][0]
        elif send.color_mask_subject_cnts[2] == 2 :
            XMax_one = send.color_mask_subject_XMax[2][0]
            XMin_one = send.color_mask_subject_XMin[2][0]
            XMin_two = send.color_mask_subject_XMin[2][1]
            XMax_two = send.color_mask_subject_XMax[2][1]

        if XMin_one > XMin_two	:				
            B_right = XMin_one
        elif XMin_one < XMin_two :
            B_right = XMin_two
        if XMax_one > XMax_two : 
            B_left = XMax_two
        elif XMax_one < XMax_two :
            B_left = XMax_one
    else : 
        red_flag = False

    for i in range (0, 32, 1):
        Filter_Matrix.append(0)
        Filter_Matrix[i] = Focus_Matrix[i] - deep.Deep_Matrix[i]
        if Filter_Matrix[i] >= 0 :
            Xc_count += 1
            Xc_num += i
            Xc = int(Xc_num) // int(Xc_count)
        else :
            Filter_Matrix[i] = 0
        WR += (32-i) * Filter_Matrix[i]
        WL += (i+1) * Filter_Matrix[i]
        if deep.Deep_Matrix[i] < Dy:
            Dy = deep.Deep_Matrix[i]
        Deep_sum += deep.Deep_Matrix[i]
        L_Deep = deep.Deep_Matrix[4]
        R_Deep = deep.Deep_Matrix[28]
    if WL > WR:
        Xb = 31
    elif WL <= WR:
        Xb = 0
    Dx = Xc - Xb
#=============================strategy=============================


def Move(Straight_status = 0 ,x = -400 ,y = -300 ,z = 0 ,theta = 2 ,sensor = 0 ):
    print('Straight_status = ' + str(Straight_status))
    if Straight_status == 0:    #speed + turn
        print('Straight_status = turn')
        send.sendContinuousValue(x + Goal_speed,y,z,theta + Angle,sensor)
    elif Straight_status == 1:     #speed ++
        print('Straight_status = go straight')
        send.sendContinuousValue(x + Goal_speed,y,z,theta,sensor)
    elif Straight_status == 2:   #max speed
        print('Straight_status = max speed')
        send.sendContinuousValue(x + 2000,y,z,theta,sensor)
    elif Straight_status == 3:  #speed + imu
        print('Straight_status = imu fix')
        send.sendContinuousValue(x + Goal_speed,y,z,theta + imu_angle,sensor)
    elif Straight_status == 4:  #left move
        print('Straight_status = left move')
        send.sendContinuousValue(x,y + 800,z,theta,sensor)
    elif Straight_status == 5:  #right move
        print('Straight_status = right move')
        send.sendContinuousValue(x,y - 800,z,theta,sensor)
    elif Straight_status == 6:  #stay
        print('Straight_status = stay')
        send.sendContinuousValue(x,y,z,theta,sensor)
    elif Straight_status == 7:  #reddoor go
        print('Straight_status = reddoor forward')
        send.sendContinuousValue(x + 500,y,z,theta,sensor)
    elif Straight_status == 8:  #reddoor back
        print('Straight_status = reddoor back')
        send.sendContinuousValue(x - 500,y,z,theta,sensor)


def Turn_Head(x = -400 ,y = -300 ,z = 0 ,theta = 2 ,sensor = 0 ):
    global R_deep_sum, L_deep_sum, L_Deep, R_Deep
    send.sendContinuousValue(x,y,z,theta,sensor)
    send.sendHeadMotor(1,1447,100)
    send.sendHeadMotor(2,1600,100)
    time.sleep(1) 
    Image_Init()
    Normal_Obs_Parameter()
    R_deep_sum = Deep_sum
    send.sendHeadMotor(1,2647,100)
    send.sendHeadMotor(2,1600,100)
    time.sleep(2)
    Image_Init()
    Normal_Obs_Parameter()
    L_deep_sum = Deep_sum
    send.sendHeadMotor(1,2048,100)
    send.sendHeadMotor(2,1500,100)
    time.sleep(1)
    print('R_deep_sum = ',R_deep_sum)
    print('L_deep_sum = ',L_deep_sum)

    if R_deep_sum > L_deep_sum :
        get_IMU()
        if abs(Yaw_wen) < 80:
            while abs(Yaw_wen) < 80:
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                send.sendContinuousValue(x + 200,y + 100,z,theta - 8,sensor)
        send.sendHeadMotor(1,2647,100)
        send.sendHeadMotor(2,1600,100) 
        time.sleep(1)
        Image_Init()
        Normal_Obs_Parameter()
        if L_Deep < 12 : #L_Deep != 24 or R_Deep != 24
            while L_Deep < 12 : #L_Deep != 24 or R_Deep != 24
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                if abs(Yaw_wen) > 87 :
                    send.sendContinuousValue(x + 2000,y,z,theta + 2,sensor)
                else :
                    send.sendContinuousValue(x + 2000,y,z,theta - 2,sensor)
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,1520,100) 
        time.sleep(1)
        get_IMU()
        if abs(Yaw_wen) > 5:
            while abs(Yaw_wen) > 5:
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                send.sendContinuousValue(x,y,z,theta + 8,sensor)
    elif L_deep_sum > R_deep_sum :
        get_IMU()
        if abs(Yaw_wen) < 80:
            while abs(Yaw_wen) < 80:
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                send.sendContinuousValue(x + 200,y + 100,z,theta + 8,sensor)
        send.sendHeadMotor(1,1447,100)
        send.sendHeadMotor(2,1600,100) 
        time.sleep(1)
        Image_Init()
        Normal_Obs_Parameter()
        if R_Deep < 12 : #L_Deep > 12 or R_Deep > 12
            while R_Deep < 12 : #L_Deep > 12 or R_Deep > 12
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                if abs(Yaw_wen) > 87 :
                    send.sendContinuousValue(x + 2000,y,z,theta - 2,sensor)
                else :
                    send.sendContinuousValue(x + 2000,y,z,theta + 2,sensor)
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,1520,100) 
        time.sleep(1)
        get_IMU()
        if abs(Yaw_wen) > 5:
            while abs(Yaw_wen) > 5:
                send.sendContinuousValue(x,y,z,theta - 8,sensor)
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()

def Slope_fix():
    # print('slope = ',deep.degree)
    pass

def IMU_Yaw_ini():
    global  Yaw_wen
    Yaw_wen = 0

def get_IMU():
    global Yaw_wen
    Yaw_wen = send.imu_value_Yaw
    print('Yaw = ' + str(Yaw_wen))
    return Yaw_wen

def IMU_Angle():
    global imu_angle
    if Yaw_wen > 0: #fix to r
        if Yaw_wen >= 45:
            imu_angle = -12
        elif 45 > Yaw_wen >= 20:
            imu_angle = -10
        elif 20 > Yaw_wen >= 10:
            imu_angle = -8
        elif 10 > Yaw_wen >= 5:
            imu_angle = -6
        elif 5 > Yaw_wen >= 2:
            imu_angle = -4
        elif 2 > Yaw_wen >= 0:
            imu_angle = 0
    elif Yaw_wen <= 0: #fix to l
        if -45 >= Yaw_wen:
            imu_angle = 12
        elif -20 >= Yaw_wen > -45:
            imu_angle = 10
        elif -10 >= Yaw_wen > -20:
            imu_angle = 8
        elif -5 >= Yaw_wen > -10:
            imu_angle = 6
        elif -2 >= Yaw_wen > -5:
            imu_angle = 4
        elif 0 >= Yaw_wen > -2:
            imu_angle = 0
    print( 'imu_angle = ' + str(imu_angle))
    return imu_angle

def Straight_Speed():
    global Goal_speed 
    if Dy ==24:
        Goal_speed = 2000
    elif 16 <= Dy < 24:
        Goal_speed = 1500
    elif 8 <= Dy < 16:
        Goal_speed = 1000
    elif 4 <= Dy < 8:
        Goal_speed = 500
    elif 0 <= Dy < 4:
        Goal_speed = 0
    print( 'Goal_speed = ' + str(Goal_speed))
    return Goal_speed

def Turn_Angle(Turn_angle_status):
    global Angle
    if Turn_angle_status == 0: #R
        print('turn right')
        if 17 > Dx >= 12:
            Angle = -13
        elif 12 > Dx >= 8:
            Angle = -11
        elif 8 > Dx >= 4:
            Angle = -9
        elif 4 > Dx >= 2:
            Angle = -7
        elif 2 > Dx >= 0:
            Angle = 0
    elif Turn_angle_status == 1: #L
        print('turn left')
        if -12 >= Dx > -17:
            Angle = 13
        elif -8 >= Dx > -12:
            Angle = 10
        elif -4 >= Dx > -8:
            Angle = 8
        elif -2 >= Dx > -4:
            Angle = 6
        elif 0 >= Dx > -2:
            Angle = 0
    else: 
        Angle = 0
    print( 'Angle = ' + str(Angle))
    return Angle

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
                #==============================image===============================
                # Focus_Matrix = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1]
                Focus_Matrix = [5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 9, 9, 10, 10, 11, 11, 11, 11, 10, 10, 9, 9, 9, 9, 8, 8, 7, 7, 6, 6, 5, 5]
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                #=============================strategy=============================
                if walking == False:
                    send.sendHeadMotor(1,2048,100)
                    send.sendHeadMotor(2,1550,100)
                    send.sendBodyAuto(0,0,0,0,1,0)
                    time.sleep(1.5) 
                walking = True
                if red_flag == True:
                    print('In Reddoor')
                    # print('Dy = ' + str(Dy))
                    get_IMU()
                    if abs(Yaw_wen) > 5:
                        while abs(Yaw_wen) > 5:
                            get_IMU()
                            IMU_Angle()
                            Move(Straight_status = 3)
                    if First_Reddoor == False :
                        First_Reddoor = True
                        send.sendHeadMotor(1,2048,100)
                        send.sendHeadMotor(2,1620,100)
                        time.sleep(0.5)
                    elif First_Reddoor == True :
                        if (Dy >= 6) and (redoor_dis == False) :
                            Move(Straight_status = 7)
                            pass
                        elif (Dy < 3) and (redoor_dis == False) :
                            Move(Straight_status = 8)
                            pass
                        else :
                            redoor_dis == True
                            if R_min < 2 and R_max > 315 :
                                print('red center')
                                if (B_min < 2 and B_max < 20) or (B_max > 315 and B_min > 300) or (B_min == 0 and B_min == 0 and B_right == 0 and B_left == 0) or (B_right > 280 and B_left < 40) :
                                    print('CCCCCCCCCCCCCCCCRWAL')
                                    Move(Straight_status = 6)
                                elif B_min < 2 and B_max > 20 :
                                    print('move R 11111')
                                    Move(Straight_status = 5)
                                elif B_max > 315 and B_min < 300 :
                                    print('move L 11111')
                                    Move(Straight_status = 4)
                            elif R_min < 2 and R_max < 315 : 
                                print('move L')
                                Move(Straight_status = 4)
                            elif R_min > 2 and R_max > 315 : 
                                print('move R')
                                Move(Straight_status = 5)
                else :
                    get_IMU()
                    if Dy < 24:
                        if 14 >= Dx > 2 :        #turn right
                            print('right avoid')
                            Straight_Speed()
                            if ( abs(Yaw_wen) > 5 and IMU_ok == False ) and Dx >= 8 :
                                IMU_Angle()
                                Move(Straight_status = 3)
                            else:
                                Turn_Angle(Turn_angle_status = 0)
                                Move(Straight_status = 0)

                            if abs(Yaw_wen) < 5 :
                                IMU_ok = True
                            # Turn(Turn_status = 0)
                        elif -2 > Dx >= -14 :     #turn left
                            print('left avoid')
                            Straight_Speed()
                            if ( abs(Yaw_wen) > 5 and IMU_ok == False ) and Dx <= -7 :
                                IMU_Angle()
                                Move(Straight_status = 3)
                            else:
                                Turn_Angle(Turn_angle_status = 1)
                                Move(Straight_status = 0)

                            if abs(Yaw_wen) < 5 :
                                IMU_ok = True
                            # Turn(Turn_status = 1)
                        elif (Dx < 17 and Dx >= 15) or (Dx <= -15 and Dx > -17) :
                            IMU_Angle()
                            if ( abs(Yaw_wen) > 5 and IMU_ok == False ) :
                                IMU_Angle()
                                Move(Straight_status = 3)
                            if abs(Yaw_wen) < 5 :
                                IMU_ok = True
                            Move(Straight_status = 6)
                            if ( L_Deep < 20 ) and ( R_Deep < 20 ) :
                                print('TTTTTTTTTTTurn Head')
                                Turn_Head()
                            else :
                                if Dx > 0 :
                                    Move(Straight_status = 0)
                                elif Dx < 0 :
                                    Move(Straight_status = 0)
                        elif (2 >= Dx >= -2) or abs(Dx) >= 17:
                            print('no avoid')
                            Move(Straight_status = 2)
                            if Dx == 0 :
                                IMU_ok = False
                    elif Dy == 24:
                        print('go straight')
                        Straight_Speed()
                        IMU_Angle()
                        Move(Straight_status = 1)
                    print('IMU_ok ====== ' + str(IMU_ok))
            print('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa ====== ' + str(Dy))
            if send.is_start == False:
                print("stop")
                if walking == True:
                    send.sendContinuousValue(0,0,0,0,0)
                    time.sleep(1.5) 
                    send.sendBodyAuto(0,0,0,0,1,0)
                    walking = False
                time.sleep(1)
                send.sendBodySector(29)
                IMU_Yaw_ini()
            print('walking ====== ' + str(walking))
            Slope_fix()
    except rospy.ROSInterruptException:
        pass

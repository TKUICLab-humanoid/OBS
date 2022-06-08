#!/usr/bin/env python
#coding=utf-8
# from turtle import st
from re import L
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
Y_Dy = 24
B_Dy = 24
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
slope_angle = 0
IMU_ok = False
Deep_sum = 0
R_deep_sum = 0
L_deep_sum = 0
L_Deep = 0
R_Deep = 0
C_Deep = 0
#----BLUE_DeepMatrix---
B_Deep_sum = 0
B_L_Deep = 0
B_R_Deep = 0
B_C_Deep = 0
#----------------------
#----Y_line DeepMatrix---
Y_Deep_sum = 0
Y_L_Deep = 0
Y_R_Deep = 0
Y_C_Deep = 0
#----------------------

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
imu_flag = True
slope_flag = True
PreTurn_L = False
# PreTurn_L = True
PreTurn_R = False
# PreTurn_R = True
crawl_flag = False
slopR_flag = False
slopL_flag = False
slope_Rcnt = 0
slope_Lcnt = 0
BR_flag = False
BL_flag = False
crawl_cnt = 0
imu_back = False
Yavoid_flag = False
Y_L_flag = False
Y_R_flag = False

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
    C_Deep = 0
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
        # print(deep.aa)
        # print(Filter_Matrix)
        # print('WR = '+ str(WR))
        # print('WL = '+ str(WL))
        # print('Xb = '+ str(Xb))
        # print('Xc_count = '+ str(Xc_count))
        # print('Xc_num = '+ str(Xc_num))
        # print('Xc = '+ str(Xc))
        print('Dx = '+ str(Dx))
        print('Dy = '+ str(Dy))
        # print('Yavoid_flag = ',Yavoid_flag)
        print('Y_L_Deep = ',Y_L_Deep)
        print('Y_R_Deep = ',Y_R_Deep)
        # print('Y_C_Deep = ',Y_C_Deep)
    else :
        print('red_flag = '+ str(red_flag))
        print('R_min = '+ str(R_min))
        print('R_max = '+ str(R_max))
        print('B_min = '+ str(B_min))
        print('B_max = '+ str(B_max))
        print('B_left = '+ str(B_left))
        print('B_right = '+ str(B_right))
        print('slopR_flag = '+ str(slopR_flag))
        print('slopL_flag = '+ str(slopL_flag))
        print('slope_Rcnt = '+ str(slope_Rcnt))
        print('slope_Lcnt = '+ str(slope_Lcnt))

def Normal_Obs_Parameter():
    global Filter_Matrix, Xc, Dy,Y_Dy,B_Dy, WR, WL, Xb, Dx, Xc_count, Xc_num, Deep_sum,Y_Deep_sum,B_Deep_sum, R_Deep, L_Deep, C_Deep,Y_R_Deep, Y_L_Deep, Y_C_Deep,B_R_Deep,B_L_Deep,B_C_Deep, red_flag, R_min, R_max, B_min, B_max, B_left, B_right, XMax_one, XMin_one, XMin_two, XMax_two
    if send.color_mask_subject_size[5][0] > 5000 :
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
#----------------Blue_DeepMatrix----------------
    for j in range (0, 32, 1):
        # Filter_Matrix.append(0)
        # Filter_Matrix[j] = Focus_Matrix[j] - deep.ya[j]
        if deep.ba[j] < B_Dy:
            B_Dy = deep.ba[j]
        B_Deep_sum += deep.ba[j]
        B_L_Deep = deep.ba[2]
        B_R_Deep = deep.ba[30]
        B_C_Deep = deep.ba[16]
#----------------Y_line_DeepMatrix----------------
    for j in range (0, 32, 1):
        # Filter_Matrix.append(0)
        # Filter_Matrix[j] = Focus_Matrix[j] - deep.ya[j]
        if deep.ya[j] < Y_Dy:
            Y_Dy = deep.ya[j]
        Y_Deep_sum += deep.ya[j]
        Y_L_Deep = deep.ya[2]
        Y_R_Deep = deep.ya[30]
        Y_C_Deep = deep.ya[16]
#--------------------------------------------------
    for i in range (0, 32, 1):
        Filter_Matrix.append(0)
        Filter_Matrix[i] = Focus_Matrix[i] - deep.aa[i]
        if Filter_Matrix[i] >= 0 :
            Xc_count += 1
            Xc_num += i
            Xc = int(Xc_num) // int(Xc_count)
        else :
            Filter_Matrix[i] = 0
        WR += (32-i) * Filter_Matrix[i]
        WL += (i+1) * Filter_Matrix[i]
        if deep.aa[i] < Dy:
            Dy = deep.aa[i]
        Deep_sum += deep.aa[i]
        L_Deep = deep.aa[4]
        R_Deep = deep.aa[28]
        C_Deep = deep.aa[16]
    if WL > WR:
        Xb = 31
    elif WL <= WR:
        Xb = 0
    Dx = Xc - Xb


#=============================strategy=============================

#-----------------------------Parameter------------------------------------
def Move(Straight_status = 0 ,x = -600 ,y = 0 ,z = 0 ,theta = 2  ,sensor = 0 ):
    print('Straight_status = ' + str(Straight_status))

    if Straight_status == 0:    #stay
        print('Straight_status = stay')
        send.sendContinuousValue(x,y,z,theta,sensor)

    elif Straight_status == 11:  #speed + turn
        print('Straight_status = turn')
        send.sendContinuousValue(x + Goal_speed,y,z,theta + Angle,sensor)

    elif Straight_status == 12:  #speed + imu
        print('Straight_status = imu fix')
        if red_flag == True:
            send.sendContinuousValue(x,y,z,theta + imu_angle,sensor)
        else:
            # send.sendContinuousValue(x + Goal_speed,y,z,theta + imu_angle,sensor)
            send.sendContinuousValue(x ,y,z,theta + imu_angle,sensor)
    elif Straight_status == 122:
        print('Straight_status = imu fix and Speed')
        send.sendContinuousValue(1500,y,z,theta + imu_angle,sensor)

    elif Straight_status == 13:  #speed ++
        print('Straight_status = go straight')
        send.sendContinuousValue(x + Goal_speed,y,z,-2,sensor)

    elif Straight_status == 155:  # turn left yellow
        print('Straight_status =  forward and left')
        send.sendContinuousValue(x ,y ,z , 10 ,sensor)

    elif Straight_status == 156:  # turn right yellow
        print('Straight_status =  forward and right')
        send.sendContinuousValue(x ,y ,z , -12 ,sensor)


#============================================================================#
#############################################################################
#============================================================================#       
    elif Straight_status == 14:  #max speed
        print('Straight_status = max speed')
        send.sendContinuousValue(2300 ,-100 ,z ,1 ,sensor)  

    elif Straight_status == 15:  # forward
        print('Straight_status =  forward')
        send.sendContinuousValue(1000 ,200 ,z ,1 ,sensor)

    elif Straight_status == 16:  # back
        print('Straight_status =  back')
        send.sendContinuousValue(-1200 ,100 ,z ,0 ,sensor)

#---------------------Turn Head Parameter-------------------------#
    elif Straight_status == 21:  #turn right
        print('Straight_status = turn right')
        send.sendContinuousValue(-200 ,500 ,z ,-10 ,sensor)

    elif Straight_status == 22:  #right turn back
        print('Straight_status = right turn back')
        send.sendContinuousValue(-200 ,-700 ,z ,11 ,sensor)

    elif Straight_status == 23:  #turn left
        print('Straight_status = turn left')
        send.sendContinuousValue(-200 ,-800 ,z ,13 ,sensor)

    elif Straight_status == 24:  #left turn back
        print('Straight_status = left turn back')
        send.sendContinuousValue(-100 ,300 ,z ,-9 ,sensor)
#--------------------turn head go straight------------------------#
    elif Straight_status == 25:  #turn right fix left
        print('Straight_status = turn right fix left')
        send.sendContinuousValue(1800 ,y ,z ,4 ,sensor)

    elif Straight_status == 26:  #turn right fix right
        print('Straight_status = turn right fix right')
        send.sendContinuousValue(1800 ,y ,z ,-3 ,sensor)

    elif Straight_status == 27:  #turn left fix right
        print('Straight_status = turn left fix right')
        send.sendContinuousValue(1800 ,y ,z ,-2 ,sensor)

    elif Straight_status == 28:  #turn left fix left
        print('Straight_status = turn left fix left')
        send.sendContinuousValue(1800 ,y ,z ,3 ,sensor) 

#------------------reddoor slope parameter------------------------#
    elif Straight_status == 31:  #Slope fix
        print('Straight_status = Slope fix')
        send.sendContinuousValue(x,y,z,theta + slope_angle,sensor)

    elif Straight_status == 32:  #slope fix right
        print('Straight_status = slope fix right')
        if send.color_mask_subject_YMax[5][0] < 70:
            rx = 200
        elif send.color_mask_subject_YMax[5][0] > 80:
            rx = -200
        else :
            rx = 0
        if abs(deep.slope)  > 0.03 or slope_flag == True:
            Slope_fix() 
        send.sendContinuousValue(-400 + rx , -1000 ,0 ,-3 + slope_angle ,0) 
    
    elif Straight_status == 33:  #slope fix left
        print('Straight_status = slope fix left')
        if send.color_mask_subject_YMax[5][0] < 70:
            rx = 200
        elif send.color_mask_subject_YMax[5][0] > 80:
            rx = -200
        else :
            rx = 0
        if abs(deep.slope)  > 0.03 or slope_flag == True:
            Slope_fix() 
        send.sendContinuousValue(-400 + rx , 800 ,0 ,3 + slope_angle ,0) 

#--------------------Preturn Head Parameter-----------------------#
    elif Straight_status == 41:  #preturn left
        print('Straight_status =preturn left')
        send.sendContinuousValue(300,y,z,13,sensor)

    elif Straight_status == 42:  #preturn right
        print('Straight_status = preturn right')
        send.sendContinuousValue(300,y,z,-7,sensor)
    
    
def Y_Line_avoid():
    global Y_L_Deep,Y_C_Deep,Y_R_Deep,Yavoid_flag,imu_back,Y_L_flag,Y_R_flag,B_C_Deep,B_L_Deep,B_R_Deep
    print('cccccccccccccccccccccsssssssssssssssssssssssssss')
    # if Y_L_Deep != 24 and Y_C_Deep > 20 and Y_R_Deep != 24:
    if (Y_L_Deep != 24 and Y_R_Deep != 24 and send.color_mask_subject_cnts[1] == 2):#黃黃 
        print('go to YY straight')
        YY_avoid()
    if Y_L_Deep < Y_R_Deep :#and Y_L_Deep < Y_R_Deep:#左轉 (Y_L_Deep > 3 and Y_R_Deep >= 20 and Y_C_Deep > 5)
        imu_back = True
        # while (Y_L_Deep > 3 and Y_C_Deep > 5):
        while ( Y_C_Deep > 1):
            if Yaw_wen <= 85:
                while Yaw_wen <= 85:
                    get_IMU()
                    Image_Init()
                    Normal_Obs_Parameter()
                    Image_Info()
                    Move(Straight_status = 23) 
                    print('turn to Yellow Line LLLLLLLLL')
            elif Yaw_wen > 85:
                get_IMU()
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                Move(Straight_status = 15) #前進
                print('close to Yellow Line LLLLLLLLLLLOOOOOOOOOOOO')
        Yavoid_flag = False
    elif Y_R_Deep < Y_L_Deep:#and Y_R_Deep < Y_L_Deep:#右轉 (Y_R_Deep > 3 and Y_L_Deep >= 20 and Y_C_Deep > 5)
        imu_back = True
        # while (Y_R_Deep > 3 and Y_C_Deep > 5):
        while (Y_C_Deep > 1):
            if Yaw_wen >= -85:
                while Yaw_wen >= -85:
                    get_IMU()
                    Image_Init()
                    Normal_Obs_Parameter()
                    Image_Info()
                    Move(Straight_status = 21) 
                    print('turn to Yellow Line RRRRRRRRRR')
            elif Yaw_wen < -85:
                get_IMU()
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                Move(Straight_status = 15) #前進
                print('close to Yellow Line RRRRRRRRRROOOOOOOOOOOO')
        Yavoid_flag = False
    else:
        # Yavoid_flag = False
        get_IMU()
        IMU_Angle()
        Image_Init()
        Normal_Obs_Parameter()
        Image_Info()
        Yavoid_flag = False
        # if abs(Yaw_wen) > 5:
        #     while abs(Yaw_wen) > 5:
        #         Move(Straight_status = 12)
        #         Yavoid_flag = False
        #         print('backbackback')
        # else:
        pass
    get_IMU()
    if abs(Yaw_wen) > 5 :
        while abs(Yaw_wen) > 5 :
            get_IMU()
            IMU_Angle()
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            if Yaw_wen > 0:
                Move(Straight_status = 24)
            elif Yaw_wen <= 0:
                Move(Straight_status = 22)
            print('iiiiiiiiiiiiii__________FFFFFFFFFFIIIIIIIXXXXXXX')
        Yavoid_flag = False
    else :
        pass
    Image_Init()
    Normal_Obs_Parameter()
    Image_Info()
    if C_Deep > 12 :
        while C_Deep > 12 :
            get_IMU()
            IMU_Angle()
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            Move(Straight_status = 122)
            print('GGGGGGGGGGGGGGGOOOOOOOOOOOOOOOOOOO')
            print('YYYYminni = ',send.color_mask_subject_YMin[1][0])
            print('FFFFFF =' , deep.line_flag)
            print('send.color_mask_subject_cnts = ',send.color_mask_subject_cnts[1])
        Yavoid_flag = False
    else :
        Yavoid_flag = False
        get_IMU()
        IMU_Angle()
        Image_Init()
        Normal_Obs_Parameter()
        Image_Info()
        print('YYYYminni = ',send.color_mask_subject_YMin[1][0])
        print('FFFFFF =' , deep.line_flag)
        print('send.color_mask_subject_cnts = ',send.color_mask_subject_cnts[1])
        pass

def YY_avoid():
    global Y_L_Deep,Y_C_Deep,Y_R_Deep,Yavoid_flag,imu_back,Y_L_flag,Y_R_flag,B_C_Deep,B_L_Deep,B_R_Deep
    get_IMU()
    Image_Info()
    # if (Y_L_Deep < 10 and Y_R_Deep < 10):
    if(Yaw_wen > 5):#左轉
        while ( Y_C_Deep > 1):
            print('YYYY Turn L')
            if Yaw_wen <= 85:
                while Yaw_wen <= 85:
                    get_IMU()
                    Image_Init()
                    Normal_Obs_Parameter()
                    Image_Info()
                    Move(Straight_status = 23) 
                    print('turn to YYY Line LLLLLLLLL')
            elif Yaw_wen > 85:
                get_IMU()
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                Move(Straight_status = 15) #前進
                print('close to YYYYY Line LLLLLLLLLLLOOOOOOOOOOOO')
    if(Yaw_wen < -5):#右轉
        while (Y_C_Deep > 1):
            print('YYYY Turn R')
            if Yaw_wen >= -85:
                while Yaw_wen >= -85:
                    get_IMU()
                    Image_Init()
                    Normal_Obs_Parameter()
                    Image_Info()
                    Move(Straight_status = 21) 
                    print('turn to YYY Line RRRRRRRRRR')
            elif Yaw_wen < -85:
                get_IMU()
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                Move(Straight_status = 15) #前進
                print('close to Yellow Line RRRRRRRRRROOOOOOOOOOOO')
    if(-5<=Yaw_wen<=5):
        pass
    get_IMU()
    if abs(Yaw_wen) > 8 :
        while abs(Yaw_wen) > 8 :
            get_IMU()
            IMU_Angle()
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            if Yaw_wen > 0:
                Move(Straight_status = 24)
            elif Yaw_wen <= 0:
                Move(Straight_status = 22)
            print('iiiiiiiMMMMuuuuFFFFFFFFFFIIIIIIIXXXXXXX')
        # Yavoid_flag = False
    else :
        pass
    Image_Init()
    Normal_Obs_Parameter()
    Image_Info()
    if C_Deep > 12 :
        while C_Deep > 12 :
            get_IMU()
            IMU_Angle()
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            Move(Straight_status = 122)
            print('GGO')
            print('CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC ====== ' + str(C_Deep))
            print('YYYYminni = ',send.color_mask_subject_YMin[1][0])
            print('FFFFFF =' , deep.line_flag)
            print('send.color_mask_subject_cnts = ',send.color_mask_subject_cnts[1])
        # Yavoid_flag = False
    else :
        # Yavoid_flag = False
        get_IMU()
        IMU_Angle()
        Image_Init()
        Normal_Obs_Parameter()
        Image_Info()
        print('YYYYminni = ',send.color_mask_subject_YMin[1][0])
        print('FFFFFF =' , deep.line_flag)
        print('send.color_mask_subject_cnts = ',send.color_mask_subject_cnts[1])
        pass

def Turn_Head():
    global R_deep_sum, L_deep_sum, L_Deep, R_Deep
    Move(Straight_status = 0)
    time.sleep(1)
    send.sendHeadMotor(1,1447,100)
    send.sendHeadMotor(2,1600,100)
    time.sleep(2) 
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
    send.sendHeadMotor(2,1550,100)
    time.sleep(1)
    print('R_deep_sum = ',R_deep_sum)
    print('L_deep_sum = ',L_deep_sum)

    if R_deep_sum > L_deep_sum :
        Image_Init()
        Normal_Obs_Parameter()
        Image_Info()
        if ( C_Deep > 4 ):
            while ( C_Deep > 4 ):
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                Move(Straight_status = 15) 
            print('TTTTTTTTTTTurn Headrrrrrrrr')
        get_IMU()
        if abs(Yaw_wen) < 80:
            while abs(Yaw_wen) < 80:
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                Move(Straight_status = 21)
        send.sendHeadMotor(1,2647,100)
        send.sendHeadMotor(2,1550,100) 
        time.sleep(1)
        Image_Init()
        Normal_Obs_Parameter()
        if abs(Dx) >=4  : #L_Deep != 24 or R_Deep != 24 / L_Deep < 12
            while abs(Dx) >= 4 : #L_Deep != 24 or R_Deep != 24 / L_Deep < 12
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                if abs(Yaw_wen) > 87 :#視步態更動
                    Move(Straight_status = 25)
                    print(' Dx = ',Dx)
                else :
                    Move(Straight_status = 26)
                    print(' Dx = ',Dx)
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,1550,100) 
        time.sleep(1)
        get_IMU()
        if abs(Yaw_wen) > 40:
            while abs(Yaw_wen) > 40:
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                Move(Straight_status = 22)
    elif L_deep_sum > R_deep_sum :
        Image_Init()
        Normal_Obs_Parameter()
        Image_Info()
        if ( C_Deep > 4 ):
            while ( C_Deep > 4 ):
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                Move(Straight_status = 15) 
            print('TTTTTTTTTTTurn Headlllllllllll')
        get_IMU()
        if abs(Yaw_wen) < 80:
            while abs(Yaw_wen) < 80:
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                Move(Straight_status = 23)
        send.sendHeadMotor(1,1447,100)
        send.sendHeadMotor(2,1550,100) 
        time.sleep(1)
        Image_Init()
        Normal_Obs_Parameter()
        if  abs(Dx) >= 4 : #L_Deep > 12 or R_Deep > 12 / R_Deep < 12  #R_Deep < 12
            while  abs(Dx) >= 4 : #L_Deep > 12 or R_Deep > 12 / R_Deep < 12
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                if abs(Yaw_wen) > 85 :
                    Move(Straight_status = 27)
                    print(' Dx = ',Dx)
                else :
                    Move(Straight_status = 28) 
                    print(' Dx = ',Dx)
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,1550,100) 
        time.sleep(1)
        get_IMU()
        if abs(Yaw_wen) > 40:
            while abs(Yaw_wen) > 40:    
                Move(Straight_status = 24)
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()

def Slope_fix():
    global slope_angle ,slope_Rcnt,slope_Lcnt,slopR_flag,slopL_flag
    print('slope = ',deep.slope)
    if deep.slope > 0: #fix to l
        if deep.slope >= 1:
            slope_angle = 13
        elif 1 > deep.slope >= 0.36:
            slope_angle = 11
        elif 0.36 > deep.slope >= 0.17:
            slope_angle = 10
        elif 0.17 > deep.slope >= 0.08:
            slope_angle = 7
        elif 0.08 > deep.slope >= 0.03:
            slope_angle = 5
        elif 0.03 > deep.slope >= 0:
            slope_angle = 0
            slope_Lcnt  += 1
        # if slope_Lcnt > 400:
        #     slopL_flag == True
        #     print('llllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllllll')
    elif deep.slope <= 0: #fix to r
        if -1 >= deep.slope:
            slope_angle = -10
        elif -0.36 >= deep.slope > -1:
            slope_angle = -9
        elif -0.17 >= deep.slope > -0.36:
            slope_angle = -7
        elif -0.08 >= deep.slope > -0.17:
            slope_angle = -6
        elif -0.03 >= deep.slope > -0.08:
            slope_angle = -5
        elif 0 >= deep.slope > -0.03:
            slope_angle = 0
            slope_Rcnt  += 1
        # if slope_Rcnt > 400:
        #     slopR_flag == True
        #     print('rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr')

    print( 'slope_angle = ' + str(slope_angle))
    return slope_angle

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
        if Yaw_wen >= 90:
            imu_angle = -17
        elif 90 > Yaw_wen >= 60:
            imu_angle = -15
        elif 60 > Yaw_wen >= 45:
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
        if -90 >= Yaw_wen:
            imu_angle = 20
        elif -60 >= Yaw_wen > -90:
            imu_angle = 18
        elif -45 >= Yaw_wen > -60:
            imu_angle = 16
        elif -20 >= Yaw_wen > -45:
            imu_angle = 14
        elif -10 >= Yaw_wen > -20:
            imu_angle = 12
        elif -5 >= Yaw_wen > -10:
            imu_angle = 8
        elif -2 >= Yaw_wen > -5:
            imu_angle = 4
        elif 0 >= Yaw_wen > -2:
            imu_angle = 0
    print( 'imu_angle = ' + str(imu_angle))
    return imu_angle

def Straight_Speed():
    global Goal_speed 
    if Dy ==24:
        Goal_speed = 2500
    elif 20 <= Dy < 24:
        Goal_speed = 2000
    elif 16 <= Dy < 20:
        Goal_speed = 1700
    elif 14 <= Dy < 16:
        Goal_speed = 1500
    elif 12 <= Dy < 14:
        Goal_speed = 1000
    elif 8 <= Dy < 12:
        Goal_speed = 700
    elif 3 <= Dy < 6:
        Goal_speed = 300
    elif 0 <= Dy < 3:
        Goal_speed = 0
    print( 'Goal_speed = ' + str(Goal_speed))
    return Goal_speed

def Turn_Angle(Turn_angle_status):
    global Angle
    if Turn_angle_status == 0: #R
        print('turn right')
        if 17 > Dx >= 12:
            Angle = -11
        elif 12 > Dx >= 8:
            Angle = -10
        elif 8 > Dx >= 6:
            Angle = -9
        elif 6 > Dx >= 4:
            Angle = -8
        elif 4 > Dx >= 2:
            Angle = -7
        elif 2 > Dx >= 0:
            Angle = 0
    elif Turn_angle_status == 1: #L
        print('turn left')
        if -12 >= Dx > -17:
            Angle = 11
        elif -8 >= Dx > -12:
            Angle = 10
        elif -6 >= Dx > -8:
            Angle = 9
        elif -4 >= Dx > -6:
            Angle = 8
        elif -3 >= Dx > -4:
            Angle = 7
        elif -2 >= Dx > -3:
            Angle = 6
        elif 0 >= Dx > -2:
            Angle = 0
    else: 
        Angle = 0
    print( 'Angle = ' + str(Angle))
    return Angle

def Crawl():
    global Straight_status ,slopL_flag ,slopR_flag,crawl_cnt,C_Deep,L_Deep,R_Deep
    if send.color_mask_subject_YMax[5][0] < 70 or send.color_mask_subject_YMax[5][0] > 80 or 70 <= send.color_mask_subject_YMax[5][0] <= 80:
        while send.color_mask_subject_YMax[5][0] < 70 or send.color_mask_subject_YMax[5][0] > 80 or 70 <= send.color_mask_subject_YMax[5][0] <= 80:
            print('ccccccccccccccccccccccccccccrrrrrrrrrrrrrrrrrr')
            print('YYYYYYYYYYYYYYYYYYYYYYYYMMMMMMMMMMMMMMMMMMAAAAAAAAAAAAAXXXXXXXXXX =',send.color_mask_subject_YMax[5][0])
            if(send.color_mask_subject_YMax[5][0] < 70):
                Move(Straight_status = 15)
                print('crawlllllll forwardddddddddd')
            elif(send.color_mask_subject_YMax[5][0] > 80):
                Move(Straight_status = 16)
                print('crawlllllll backkkkkkkk')
            elif 70 <= send.color_mask_subject_YMax[5][0] <= 80:
                print('CCCCCCCCCCCCCCCCRWAL')
                print('rcnt = ',slope_Rcnt)
                print('lcnt = ',slope_Lcnt)
                send.sendBodyAuto(0,0,0,0,1,0)
                time.sleep(3)
                send.sendBodySector(29)
                time.sleep(3)
                send.sendBodySector(123)
                time.sleep(8)
                time.sleep(0.5)
                while crawl_cnt < 3:
                    send.sendBodySector(456)
                    time.sleep(3)
                    time.sleep(0.2)
                    crawl_cnt += 1
                send.sendHeadMotor(1,2048,100)
                send.sendHeadMotor(2,2500,100)
                time.sleep(1)
                while crawl_cnt < 10:
                    Image_Init()
                    Normal_Obs_Parameter()
                    print('CCCCCCCCCCCCCCCCCCCCC = ',C_Deep)
                    print('LLLLLLLLLLLLLLLLLLLLL = ',L_Deep)
                    print('RRRRRRRRRRRRRRRRRRRRR = ',R_Deep)
                    if C_Deep >= 16 and L_Deep >= 16 and R_Deep >= 16:
                        send.sendBodySector(456)
                        time.sleep(3)
                        time.sleep(0.2)
                        crawl_cnt += 1
                    else:
                        break
                send.sendBodySector(789)
                time.sleep(13)
                time.sleep(0.5)
                send.sendBodySector(29)
                time.sleep(1)
                send.sendHeadMotor(1,2048,100)
                send.sendHeadMotor(2,1550,100)
                time.sleep(0.5)
                send.sendBodySector(15)
                time.sleep(1)
                send.sendBodyAuto(0,0,0,0,1,0)
                time.sleep(1)
                break
            # print('CCCCCCCCCCCCCCCCRWAL')
            # send.sendBodyAuto(0,0,0,0,1,0)
            # time.sleep(10)

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
                Focus_Matrix = [7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 10, 10, 10, 10, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7]#6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                #=============================strategy=============================
                if walking == False:
                    send.sendHeadMotor(1,2048,100)
                    send.sendHeadMotor(2,1550,100)
                    send.sendBodySector(15)#收手
                    # send.sendBodySector(1218)#長腳
                    time.sleep(1.5)
                    send.sendBodyAuto(0,0,0,0,1,0)
                    time.sleep(1.5) 
                walking = True
                if PreTurn_L == True :
                    get_IMU()
                    if abs(Yaw_wen) < 83:
                        while abs(Yaw_wen) < 83:
                            get_IMU()
                            Move(Straight_status = 41)
                    PreTurn_L = False
                elif PreTurn_R == True : 
                    get_IMU()
                    if abs(Yaw_wen) < 80:
                        while abs(Yaw_wen) < 80:
                            get_IMU()
                            Move(Straight_status = 42)
                    PreTurn_R = False
                if red_flag == True:
                    print('In Reddoor')
                    # print('Dy = ' + str(Dy))
                    if First_Reddoor == False :
                        First_Reddoor = True
                        send.sendHeadMotor(1,2048,100)
                        send.sendHeadMotor(2,1800,100)
                        time.sleep(0.5)
                    elif First_Reddoor == True :
                        # print('DDDDDDDDDDDDDDDDDDDDDDDDDDDDDDYYYYYYYYYYYYYYYYYYYYY = ',Dy)
                        # print('BBBBBBBBBBBBBBBBBBB___LLLLLLLLLLLLLL = ',B_left)
                        # print('BBBBBBBBBBBBBBBBBBB___RRRRRRRRRRRRRR = ',B_right)
                        print('YYYYYYYYYYYYYYYMMMMMMMMMIIIIIIIIINNNNNNNNN = ',send.color_mask_subject_YMax[5][0])
                        print('slope = ',deep.slope)

                        if abs(deep.slope) > 0.08 and (slope_flag == True):
                            # send.sendHeadMotor(1,2048,100)
                            # send.sendHeadMotor(2,1800,100)
                            # time.sleep(0.5)
                            # while abs(deep.slope) > 0.03 or slopR_flag == False or slopL_flag == False:
                            # while slopR_flag == False or slopL_flag == False:
                            # while abs(deep.slope) > 0.03 and (slope_flag == True):
                            while abs(deep.slope) > 0.08 or slope_flag == True:
                                Slope_fix()
                                Move(Straight_status = 31)
                                Image_Info()
                                print('crawl3333333333333333333333333333333333')
                                # if (slopR_flag == True) or (slopL_flag ==True):
                                if slope_Rcnt > 400 or slope_Lcnt > 400:
                                    break
                                # if -0.03 < deep.slope < 0.03 :
                                #     slope_cnt += 1
                                # else :
                                #     slope_cnt = 0
                                # if slope_cnt > 2 :
                                #     break
                            slope_flag = False
                            slope_Lcnt = 0
                            slope_Rcnt = 0
                            send.sendHeadMotor(1,2048,100)
                            send.sendHeadMotor(2,1700,100)
                            time.sleep(0.5)
                        else :
                            if (send.color_mask_subject_YMax[5][0] < 70) and (redoor_dis == False) :
                                Move(Straight_status = 15)
                                pass
                            elif (send.color_mask_subject_YMax[5][0] > 80) and (redoor_dis == False) :
                                Move(Straight_status = 16)
                                pass
                            else :
                                redoor_dis = True
                                if R_min < 2 and R_max > 315 :
                                    print('red center')
                                    print('BBBBBBBRRRRRRRRRRRRFFFFFFLLLLLLLAAAAAAAGGGGG = ',BR_flag)
                                    print('BBBLLLLLLLLLLLLLLLLFFFFFFFFFFFFFFFF = ',BL_flag)
                                    if (B_max == 0 and B_min == 0 and B_left <= 65 and BR_flag == True) or (B_max == 0 and B_min == 0 and B_right > 285 and BL_flag == True) or (B_max == 0 and B_min == 0 and B_right == 0 and B_left == 0):
                                        if abs(deep.slope)  > 0.03 or slope_flag == True:
                                            # while abs(deep.slope) > 0.03 or slopR_flag == False or slopL_flag == False:
                                            while abs(deep.slope) > 0.03 or slope_flag == True:
                                                Slope_fix()
                                                Move(Straight_status = 31)
                                                print('crawl22222222222222222222222222222222')
                                                Image_Info()
                                                # print('r = ',slopR_flag)
                                                # print('l = ',slopL_flag)
                                                # if deep.slope < 0.03:
                                                #     # slopR_flag == True
                                                #     slope_Rcnt += 1
                                                # if deep.slope < -0.03:
                                                    # slopL_flag == True
                                                    # slope_Lcnt += 1
                                                # if slopR_flag == True and slopL_flag ==True:
                                                # if (slopR_flag == True) or (slopL_flag ==True):
                                                if slope_Rcnt > 400 or slope_Lcnt > 400:
                                                    break
                                                # if -0.03 < deep.slope < 0.03 :
                                                #     slope_cnt += 1
                                                # else :
                                                #     slope_cnt = 0
                                                # if slope_cnt > 2 :
                                                #     break
                                        else :
                                            Crawl()
                                        # Move(Straight_status = 6)
                                    elif (B_min < 2 and B_max > 20):# or (B_left > 50 and B_right >= 270)
                                        print('move R 11111')
                                        BR_flag = True
                                        BL_flag = False
                                        Move(Straight_status = 32)
                                    elif (B_max > 315 and B_min < 300):# or (B_left <= 55 and B_right < 270)
                                        print('move L 11111')
                                        BL_flag = True
                                        BR_flag = False
                                        Move(Straight_status = 33)
                                    else :
                                        if BR_flag == True:
                                            Move(Straight_status = 32)
                                            print('BBBBBBBBBBBBBBRRRRRRRRRRR =',BR_flag)
                                        elif BL_flag == True:
                                            Move(Straight_status = 33)
                                            print('BBBBBBBBBBLLLLLLLLLLLLLLLLLLLL = ',BL_flag)
                                elif R_min < 2 and R_max < 315 : 
                                    print('move L')
                                    Move(Straight_status = 33)
                                elif R_min > 2 and R_max > 315 : 
                                    print('move R')
                                    Move(Straight_status = 32)
                else :
                    get_IMU()
                    if Dy < 24:
                        if 14 > Dx > 1 :        #turn right
                            print('right avoid')
                            Straight_Speed()
                            if ((deep.line_flag == True) and (send.color_mask_subject_YMin[1][0] <= 10)) or ((send.color_mask_subject_cnts[1] == 2) and (Y_L_Deep <= 7) and (Y_R_Deep <= 7)):
                                print('YYYline___right')
                                # Yavoid_flag = True
                                Y_Line_avoid()
                            # if (Y_L_Deep != 24 and Y_R_Deep != 24):
                            #     YY_avoid()
                            if (send.color_mask_subject_YMax[2][0] >= 170) and ( abs(Yaw_wen) <= 3 ) and imu_back == False and abs(Dx) > 3 and C_Deep != 24:
                                while (send.color_mask_subject_YMax[2][0] >= 170):
                                    print('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
                                    Image_Init()
                                    Normal_Obs_Parameter()
                                    Image_Info()
                                    Move(Straight_status = 16) 
                                    print('imu fix back')
                                imu_back = True
                            if ( abs(Yaw_wen) > 3 and IMU_ok == False ) and Dx >= 5 :
                                IMU_Angle()
                                Move(Straight_status = 12)
                                Image_Init()
                                Normal_Obs_Parameter()
                                Image_Info()
                            # if (abs(Yaw_wen) < 5 and IMU_ok == True):
                            #     if  (Y_L_Deep != 24 and C_Deep == 24) or  (Y_R_Deep != 24 and C_Deep == 24) or(Y_L_Deep != 24 and Dy< 5) or  (Y_R_Deep != 24 and Dy <5 ):
                            #         while (Dy < 5 or C_Deep == 24):
                            #             Image_Init()
                            #             Normal_Obs_Parameter()
                            #             Image_Info()
                            #             Move(Straight_status = 16) 
                            #             print('imu fix back')
                            else:
                                Turn_Angle(Turn_angle_status = 0)
                                Move(Straight_status = 11)

                            if abs(Yaw_wen) < 3 :
                                IMU_ok = True
                            # Turn(Turn_status = 0)
                        elif -1 > Dx > -14 :     #turn left
                            print('left avoid')
                            Straight_Speed()
                            if ((deep.line_flag == True) and (send.color_mask_subject_YMin[1][0] <= 10)) or ((send.color_mask_subject_cnts[1] == 2) and (Y_L_Deep <= 7) and (Y_R_Deep <= 7)):
                                print('YYYline___left')
                                # Yavoid_flag = True
                                Y_Line_avoid()
                            # if (Y_L_Deep != 24 and Y_R_Deep != 24):
                            #     YY_avoid()
                            if (send.color_mask_subject_YMax[2][0] >= 170) and ( abs(Yaw_wen) <= 3 ) and imu_back == False and abs(Dx) > 3 and C_Deep != 24:
                                while (send.color_mask_subject_YMax[2][0] >= 170):
                                    print('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
                                    Image_Init()
                                    Normal_Obs_Parameter()
                                    Image_Info()
                                    Move(Straight_status = 16) 
                                    print('imu fix back')
                                imu_back = True
                            if ( abs(Yaw_wen) > 3 and IMU_ok == False ) and Dx <= -7 :
                                IMU_Angle()
                                Move(Straight_status = 12)
                                Image_Init()
                                Normal_Obs_Parameter()
                                Image_Info()
                            # if (abs(Yaw_wen) < 5 and IMU_ok == True):
                            #     if  (Y_L_Deep != 24 and C_Deep == 24) or  (Y_R_Deep != 24 and C_Deep == 24) or(Y_L_Deep != 24 and Dy< 5) or  (Y_R_Deep != 24 and Dy <5 ):
                            #         while (Dy < 5 or C_Deep == 24):
                            #             Image_Init()
                            #             Normal_Obs_Parameter()
                            #             Image_Info()
                            #             Move(Straight_status = 16) 
                            #             print('imu fix back')
                            # if ( Dy < 7 ) :
                            #     flag = True
                            # else :
                            #     flag = False
                            # if (flag == True) and (abs(Yaw_wen) < 3):
                            #     print('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
                            #     while ( Dy < 7 ):
                            #         Image_Init()
                            #         Normal_Obs_Parameter()
                            #         Image_Info()
                            #         Move(Straight_status = 16) 
                            #         print('imu fix back')
                            else:
                                Turn_Angle(Turn_angle_status = 1)
                                Move(Straight_status = 11)

                            if abs(Yaw_wen) < 3 :
                                IMU_ok = True
                            # Turn(Turn_status = 1)
                        elif (Dx < 17 and Dx >= 14) or (Dx <= -14 and Dx > -17) :
                            IMU_Angle()
                            if ((deep.line_flag == True) and (send.color_mask_subject_YMin[1][0] <= 10)) or ((send.color_mask_subject_cnts[1] == 2) and (Y_L_Deep <= 7) and (Y_R_Deep <= 7)):
                                print('YYYline')
                                # Yavoid_flag = True
                                Y_Line_avoid()
                            # if (Y_L_Deep != 24 and Y_R_Deep != 24):
                            #     YY_avoid()
                            if (send.color_mask_subject_YMax[2][0] >= 170) and ( abs(Yaw_wen) <= 3 ) and imu_back == False and abs(Dx) > 3 and C_Deep != 24:
                                while (send.color_mask_subject_YMax[2][0] >= 170):
                                    print('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
                                    Image_Init()
                                    Normal_Obs_Parameter()
                                    Image_Info()
                                    Move(Straight_status = 16) 
                                    print('imu fix back')
                                imu_back = True
                            if ( abs(Yaw_wen) > 3 and IMU_ok == False ) :
                                IMU_Angle()
                                Move(Straight_status = 12)
                                Image_Init()
                                Normal_Obs_Parameter()
                                Image_Info()
                            # if (abs(Yaw_wen) < 3 and IMU_ok == True):
                            #     if  (Y_L_Deep != 24 and C_Deep == 24) or  (Y_R_Deep != 24 and C_Deep == 24) or(Y_L_Deep != 24 and Dy< 7) or  (Y_R_Deep != 24 and Dy <7 ):
                            #         while (Dy < 7 or C_Deep == 24):
                            #             Image_Init()
                            #             Normal_Obs_Parameter()
                            #             Image_Info()
                            #             Move(Straight_status = 16) 
                            #             print('imu fix back')
                                # if ( Dy < 7 ) and (abs(Yaw_wen) <= 3):
                                #     print('aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa')
                                #     while ( Dy < 7 ):
                                #         Image_Init()
                                #         Normal_Obs_Parameter()
                                #         Image_Info()
                                #         Move(Straight_status = 16) 
                                #         print('imu fix back')
                            if abs(Yaw_wen) < 3 :
                                IMU_ok = True
                            if abs(Yaw_wen) < 3:
                                if ( L_Deep < 15 ) and ( R_Deep < 15 ) and ( C_Deep < 20 ):
                                    # Move(Straight_status = 6)
                                    # if (C_Deep -R_Deep)  and (C_Deep-L_Deep)
                                    Turn_Head()
                                else :
                                    print('xXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXxxxxxxxxxxxxxxxxxxxxxxxxxx')
                                    if Dx > 0 :
                                        Move(Straight_status = 11)
                                    elif Dx < 0 :
                                        Move(Straight_status = 11)
                            else:
                                print('IMU NOT OK')
                                pass
                        elif (1 >= Dx >= -1) or abs(Dx) >= 17:
                            print('no avoid')
                            Move(Straight_status = 14)
                            if Dx == 0 :
                                IMU_ok = False
                                imu_back = False
                    elif Dy == 24:
                        print('go straight')
                        Straight_Speed()
                        #IMU_Angle()
                        Move(Straight_status = 13)
                    print('IMU_ok ====== ' + str(IMU_ok))
            # print('BBBBBBBBBBBBBBBBBBB___LLLLLLLLLLLLLL = ',B_left)
            # print('BBBBBBBBBBBBBBBBBBB___RRRRRRRRRRRRRR = ',B_right)
            # print('LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL ====== ' + str(L_Deep))
            # print('RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR ====== ' + str(R_Deep))
            # print('CCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCCC ====== ' + str(C_Deep))
            print('YYYYminni = ',send.color_mask_subject_YMin[1][0])
            print('FFFFFF =' , deep.line_flag)
            # print('Dy = ' +str(Dy))
            print('send.color_mask_subject_cnts = ',send.color_mask_subject_cnts[1])
            
            
            
            if send.is_start == False:
                print("stop")
                if walking == True:
                    send.sendContinuousValue(0,0,0,0,0)
                    time.sleep(1.5) 
                    send.sendBodyAuto(0,0,0,0,1,0)
                    time.sleep(1)
                    send.sendBodySector(29)
                    walking = False
                # time.sleep(1)
                # send.sendBodySector(29)
                IMU_Yaw_ini()
            print('walking ====== ' + str(walking))
    except rospy.ROSInterruptException:
        pass
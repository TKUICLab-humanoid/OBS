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
import curses
from std_msgs.msg import String
status = "0_stay"
YYDS = 0
CRMin = 0
CRMax = 0
Filter_Matrix = []
Xc = 0
Dy = 24
Y_Dy = 24
B_Dy = 24
WR = 0
B_Deep_sum1 = 0
B_Deep_sum2 = 0
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
Y_Deep_sum1 = 0
Y_Deep_sum2 = 0
YR_Deep_sum = 0
YL_Deep_sum = 0
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
Y_left = 0
Y_right = 0
Y_XMax_one = 0
Y_XMin_one = 0
Y_XMin_two = 0
Y_XMax_two = 0
Y_min = 0
Y_max = 0
First_Reddoor = False
redoor_dis = False
imu_flag = True
slope_flag = True

crawl_flag = False
slope_Rcnt = 0
slope_Lcnt = 0
BR_flag = False
BL_flag = False
crawl_cnt = 0
imu_back = False
Y_L_flag = False
Y_R_flag = False
L_line = False
R_line = False
y_move = 0
B_LC_Deep = 0
B_RC_Deep = 0
function = "我還沒過喔！"
x_move = 0
y_wave = 0
turn_right_x = 0
turn_right_y = 0
right_turn_back_x = 0
right_turn_back_y = 0
turn_left_x = 0
turn_left_y = 0
left_turn_back_x = 0 
left_turn_back_y = 0
R_L_Deep = 0
R_R_Deep = 0
y_move_turn = 0

#==============================image===============================
def Image_Init():
    global function,Filter_Matrix, Xc, Dy, WR, WL, Xb, Dx, Xc_count, Xc_num, Deep_sum,Y_Deep_sum,Y_Deep_sum1, C_Deep,Y_Deep_sum2, L_Deep, R_Deep, R_min, R_max, B_min, B_max, B_left, B_right, XMax_one, XMin_one, XMin_two, XMax_two, y_move,Y_min, Y_max, Y_left, Y_right, Y_XMax_one, Y_XMin_one, Y_XMin_two, Y_XMax_two
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
    Y_Deep_sum = 0
    Y_Deep_sum1 = 0
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
    Y_left = 0
    Y_right = 0
    Y_XMax_one = 0
    Y_XMin_one = 0
    Y_XMin_two = 0
    Y_XMax_two = 0
    Y_min = 0
    Y_max = 0
    y_move = 0
    function = "你猜它過了沒？ 呵呵"
    
    
def update_values():#更新數值
    global C_Deep,function,B_RC_Deep,B_LC_Deep,Y_L_Deep,Goal_speed,Deep_sum,red_flag,slope_angle,status,L_line,R_line,Yaw_wen,Y_Deep_sum1,Y_Deep_sum2,B_min,B_max,B_left, B_right,crawl_cnt,R_deep_sum,L_deep_sum,Dy,Dx,B_C_Deep,B_R_Deep,B_L_Deep,Angle
    # 移動光標到終端機的第一行
    # print("\033[H", end="")
    sys.stdout.write("\033[H")
    sys.stdout.write("\033[J")
    print("\033[1;31;40mDx and Dy\033[0m\t\033[1;31;40mTurn&&Speed\033[0m\t\033[1;31;40mIMU and Status\033[0m\n{} : {:<5}\t{}  : {:<5}\t{} : {:<5}\n{} : {:<5}\t{} : {:<5}\t{} :{:<5}\n=====================================".format("Dx",Dx,"Turn",Angle,"Yaw_wen",Yaw_wen,"Dy",Dy,"Speed",Goal_speed,"Walking_Status",status))
    
    print("\033[1;31;40mB_OBS\033[0m\t\t\033[1;31;40mY_OBS\033[0m\t\t\033[1;31;40mR_OBS\033[0m")
    print("{} : {:<5}  {} : {:<5}\t{} :{:<5}\n=====================================".format("B_YMax",send.color_mask_subject_YMax[2][0],"Y_YMax",send.color_mask_subject_YMax[1][0],"R_YMax",send.color_mask_subject_YMax[5][0]))
    print("\033[1;31;40mRed_Door_Parameter\033[0m\t<< Red_Flag : {} >>".format(red_flag))
    print("{}  : {:<5}\t{}  : {:<5}\n{} : {:<5}\t{} : {:<5}\n{} : {:<5}\t{} : {:<5}\t{} : {:<5}\n{} : {:<5}\t{} : {:<5}\n=====================================".format("B_min",B_min,"B_max",B_max,"XMin_1",XMin_one,"XMax_1",XMin_two,"XMin_2",XMax_one,"XMax_2",XMax_two,"Slope_angle",deep.slope,"B_Left",B_left,"B_Right",B_right))
    send.color_mask_subject_XMin[2][0]
    print("\033[1;31;40mY_Line Flag\033[0m\nR_line : {}\t\tR_Y_Deep : {}\nL_Line : {}\t\tL_Y_Deep : {}\n=====================================".format(R_line,Y_Deep_sum2,L_line,Y_Deep_sum1))
    print("Deep_sum: {}".format(Deep_sum))
    print("喔！ {}".format(function))
    print("==================================")
    print("C_Deep: {}".format(C_Deep))
    print("B_LC_Deep: {}".format(B_LC_Deep))
    print("B_RC_Deep: {}".format(B_RC_Deep))
    print("Y_L_Deep: {}".format(Y_L_Deep))
    print("==================================")
    print("BCDeep: {}".format(B_C_Deep))
    print("BLDeep: {}".format(B_L_Deep))
    print("BRDeep: {}".format(B_R_Deep))
    print("==================================")
    print("L_line: {}".format(L_line))
    print("R_line: {}".format(R_line))
    print("==================================")
    # print("Y1: {}".format(send.color_mask_subject_YMax[1][0]))
    # print("Y2: {}".format(send.color_mask_subject_YMax[1][1]))
    # print("Y : {}".format(send.color_mask_subject_cnts[1]))
    # print("WL : {}".format(WL))
    # print("WR : {}".format(WR))
    # print("R_YMax: {}".format(send.color_mask_subject_YMax[5][0]))
    # print("Deep_sum : {}".format(Deep_sum))
    print("R_XMid: {}".format(send.color_mask_subject_X [5][0]))
    print("R_R_Deep: {}".format(R_R_Deep))
    print("R_XMin: {}".format(send.color_mask_subject_XMin[5][0]))
    print("R_XMax: {}".format(send.color_mask_subject_XMax[5][0]))
    
def Image_Info():
    # print('==============================================================')
    # if red_flag == False:
    #     print('Dx = '+ str(Dx))
    #     print('Dy = '+ str(Dy))
    #     print('Y_L_Deep = ',Y_L_Deep)
    #     print('Y_R_Deep = ',Y_R_Deep)
    #     # print('Y_C_Deep = ',Y_C_Deep)
    # else :
    #     print('red_flag = '+ str(red_flag))
    #     print('R_min = '+ str(R_min))
    #     print('R_max = '+ str(R_max))
    #     print('B_min = '+ str(B_min))
    #     print('B_max = '+ str(B_max))
    #     print('B_left = '+ str(B_left))
    #     print('B_right = '+ str(B_right))
    #     print('slope_Rcnt = '+ str(slope_Rcnt))
    #     print('slope_Lcnt = '+ str(slope_Lcnt))
    pass

def Normal_Obs_Parameter():
    global Filter_Matrix, Xc, Dy,Y_Dy,B_Dy, WR, WL, Xb, Dx, Xc_count, Xc_num, Deep_sum,Y_Deep_sum,Y_Deep_sum1,Y_Deep_sum2,B_Deep_sum1,B_Deep_sum2,B_Deep_sum,B_RC_Deep, R_Deep, L_Deep,B_LC_Deep, C_Deep,Y_R_Deep, Y_L_Deep, Y_C_Deep,B_R_Deep,B_L_Deep,B_C_Deep, red_flag, R_min, R_max, B_min, B_max, B_left, B_right, XMax_one, XMin_one, XMin_two, XMax_two,Y_min, Y_max, Y_left, Y_right, Y_XMax_one, Y_XMin_one, Y_XMin_two, Y_XMax_two, R_L_Deep, R_R_Deep
    Focus_Matrix = [0, 0, 0, 0, 4, 4, 5, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 5, 4, 4, 0, 0, 0, 0]
    if send.color_mask_subject_size[5][0] > 5000 :                      #有紅時計算紅門資訊
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
    for j in range (0, 32, 1):                  #藍色深度
        if deep.ba[j] < B_Dy:
            B_Dy = deep.ba[j]
        B_Deep_sum += deep.ba[j]
        B_L_Deep = deep.ba[2]
        B_R_Deep = deep.ba[30]
        B_C_Deep = deep.ba[16]
        B_LC_Deep = deep.ba[8]
        B_RC_Deep = deep.ba[24]
    for o in range (0, 15, 1):                  #黃色深度L
        B_Deep_sum1 += deep.ya[o]
    for p in range (16, 32, 1):                 #黃色深度R
        B_Deep_sum2 += deep.ya[p]
    R_L_Deep = deep.ra[2]                       #紅色深度L
    R_R_Deep = deep.ra[30]                      #紅色深度R
#----------------Y_line_DeepMatrix----------------
    for j in range (0, 32, 1):                  #黃色深度
        if deep.ya[j] < Y_Dy:
            Y_Dy = deep.ya[j]
        Y_Deep_sum += deep.ya[j]
        Y_L_Deep = deep.ya[5] 
        Y_R_Deep = deep.ya[30]
        Y_C_Deep = deep.ya[16]
    for k in range (0, 15, 1):                  #黃色深度L
        Y_Deep_sum1 += deep.ya[k]
    for l in range (16, 32, 1):                 #黃色深度R
        Y_Deep_sum2 += deep.ya[l]
#--------------------------------------------------
    if send.color_mask_subject_cnts[1] == 1 :
        Y_min = send.color_mask_subject_XMin[1][0]
        Y_max = send.color_mask_subject_XMax[1][0]
    elif send.color_mask_subject_cnts[1] == 2 :
        Y_XMax_one = send.color_mask_subject_XMax[1][0]
        Y_XMin_one = send.color_mask_subject_XMin[1][0]
        Y_XMin_two = send.color_mask_subject_XMin[1][1]
        Y_XMax_two = send.color_mask_subject_XMax[1][1]

    # if Y_XMin_one > Y_XMin_two	:				
    #     Y_right = Y_XMin_one
    # elif Y_XMin_one < Y_XMin_two :
    #     Y_right = Y_XMin_two
    # if Y_XMax_one > Y_XMax_two : 
    #     Y_left = Y_XMax_two
    # elif Y_XMax_one < Y_XMax_two :
    #     Y_left = Y_XMax_one
#--------------------------------------------------
    for i in range (0, 32, 1):                      #濾波矩陣
        Filter_Matrix.append(0)
        Filter_Matrix[i] = Focus_Matrix[i] - deep.aa[i]
        if Filter_Matrix[i] >= 0 :
            Xc_count += 1
            Xc_num += i
            Xc = int(Xc_num) // int(Xc_count)
        else :
            Filter_Matrix[i] = 0
        # if WR > 0 and WL > 0 :
        #     WR += ((32-i) * Filter_Matrix[i]) + (23 - deep.aa[0])
        #     WL += ((i+1) * Filter_Matrix[i]) + (23 - deep.aa[31])
        WR += ((32-i) * Filter_Matrix[i])
        WL += ((i+1) * Filter_Matrix[i])
        if deep.aa[i] < Dy:
            Dy = deep.aa[i]
        Deep_sum += deep.aa[i]
        L_Deep = deep.aa[0]
        R_Deep = deep.aa[31]
        C_Deep = deep.aa[16]
    if WL > WR:
        Xb = 31
    elif WL <= WR:
        Xb = 0
    if (send.color_mask_subject_cnts[1] == 2) and (send.color_mask_subject_YMax[1][0] >= 200) and (send.color_mask_subject_YMax[1][1] >= 200):
        Dx = 0
    else :
        Dx = Xc - Xb
    update_values()
#-----------------------------Parameter------------------------------------
def Move(Straight_status = 0 ,x = -1100 ,y = -300 ,z = 0 ,theta = 0  ,sensor = 0 ):
    # print('Straight_status = ' + str(Straight_status))
    global status, turn_right_x, turn_right_y, turn_left_x, turn_left_y, right_turn_back_x, right_turn_back_y, left_turn_back_x, left_turn_back_y
    turn_right_x = -1400 #21 theta = -6
    turn_right_y = 600
    right_turn_back_x = -1700 #22 theta = 7
    right_turn_back_y = -1500
    turn_left_x = -1500 #23 theta = 7
    turn_left_y = -1600
    left_turn_back_x = -1300 #24 theta = -6
    left_turn_back_y = 900
    
    if Straight_status == 0:            #stay
        # print('Straight_status = stay')
        send.sendContinuousValue(x,y,z,theta,sensor)
        status = "0_Stay"

    elif Straight_status == 11:         #speed + turn  14 -9
        # print('Straight_status = turn')
        if (send.color_mask_subject_YMax[1][0] >= 220) : 
            send.sendContinuousValue(x + 200,y,z,theta + Angle,sensor)
        else:
            send.sendContinuousValue(x + Goal_speed,y,z,theta + Angle,sensor)
        status = "11_Turn"
    elif Straight_status == 12:         #speed + imu
        # print('Straight_status = imu fix')
        status = "12_Imu_Fix"
        if red_flag == True:
            send.sendContinuousValue(x-400,y,z,theta + imu_angle,sensor)
        else:
            # send.sendContinuousValue(x + Goal_speed,y,z,theta + imu_angle,sensor)
            if Yaw_wen > 0: #-13修右 -8
                #send.sendContinuousValue(x-200,y+300 ,z,theta + imu_angle,sensor)
                send.sendContinuousValue(left_turn_back_x,left_turn_back_y ,z,theta + imu_angle,sensor) #24
                # print('imu fix rightttttttttttttttt')
                status = "12_Imu_Fix_right"
            elif Yaw_wen <= 0: #8修左 
                #send.sendContinuousValue(x-200,y-300 ,z,theta + imu_angle,sensor)
                send.sendContinuousValue(right_turn_back_x,right_turn_back_y ,z,theta + imu_angle,sensor) #22
                # print('imu fix lefttttttttttttttttttt')
                status = "12_Imu_Fix_left"
    elif Straight_status == 122:        #YLine straight
        # print('Straight_status = imu fix and Speed')
        status = "122_Imu_Fix_Max_Speed"
        send.sendContinuousValue(2500,y -100,z,theta + imu_angle,sensor)

    elif Straight_status == 13:         #speed ++
        # print('Straight_status = go straight')
        status = "13_Go_Straight"
        send.sendContinuousValue(x + Goal_speed,y,z,-2,sensor)

#============================================================================#       
    elif Straight_status == 14:  #max speed
        # print('Straight_status = max speed')
        status = "14_Max_Speed"
        send.sendContinuousValue(3300 ,-100 ,z , 0,sensor)

    elif Straight_status == 15:  # small forward
        # print('Straight_status =  small forward')
        status = "15_Small_Forward"
        Slope_fix()
        send.sendContinuousValue(1500 ,-100 ,z ,0 + slope_angle,sensor)

    elif Straight_status == 16:  #small back
        # print('Straight_status =  small back')
        status = "16_Small_Back"
        Slope_fix()
        send.sendContinuousValue(-2300 ,-300 ,z ,0 + slope_angle ,sensor)

#---------------------Turn Head Parameter-------------------------#
    elif Straight_status == 21:  #turn right
        # print('Straight_status = turn right')
        status = "21_Turn_Right"
        send.sendContinuousValue(turn_right_x ,turn_right_y ,z ,-6 ,sensor)

    elif Straight_status == 22:  #right turn back
        # print('Straight_status = right turn back')
        status = "22_Turn_Right_Back"
        send.sendContinuousValue(right_turn_back_x ,right_turn_back_y ,z ,7 ,sensor)

    elif Straight_status == 23:  #turn left
        # print('Straight_status = turn left')
        status = "23_Turn_Left"
        send.sendContinuousValue(turn_left_x ,turn_left_y ,z ,7 ,sensor)

    elif Straight_status == 24:  #left turn back
        # print('Straight_status = left turn back')
        status = "24_left turn back"
        send.sendContinuousValue(left_turn_back_x ,left_turn_back_y ,z ,-6 ,sensor)

#--------------------turn head go straight------------------------#
    elif Straight_status == 25:  #turn right fix left
        # print('Straight_status = turn right fix left')
        status = "25_Turn_Right_Fix_Left"
        send.sendContinuousValue(1800 ,y + y_move ,z ,3 ,sensor)

    elif Straight_status == 26:  #turn right fix right
        # print('Straight_status = turn right fix right')
        status = "26_Turn_Right_Fix_Right"
        send.sendContinuousValue(1800 ,y + y_move ,z ,-4 ,sensor)

    elif Straight_status == 27:  #turn left fix right
        # print('Straight_status = turn left fix right')
        status = "27_Turn_Left_Fix_Right"
        send.sendContinuousValue(1800 ,y + y_move ,z ,-2 ,sensor)

    # elif Straight_status == 277:  #turn left fix right move
    #     print('Straight_status = turn left fix right move')
    #     send.sendContinuousValue(x ,y + y_move ,z ,-3 ,sensor)

    elif Straight_status == 28:  #turn left fix left
        # print('Straight_status = turn left fix left')
        status = "28_Turn_Left_Fix_Left"
        send.sendContinuousValue(1800 ,y + y_move ,z ,5 ,sensor) 

#------------------reddoor slope parameter------------------------#
    elif Straight_status == 31:  #Slope fix
        # print('Straight_status = Slope fix')
        status = "31_Slope_Fix"
        send.sendContinuousValue(x,y,z,theta + slope_angle,sensor)

    elif Straight_status == 32:  #slope fix right
        # print('Straight_status = slope fix right')
        status = "32_Slope_Fix_Right"
        if send.color_mask_subject_YMax[5][0] < 150:
            rx = 200
        elif send.color_mask_subject_YMax[5][0] > 160:
            rx = -200
        else :
            rx = 0
        if abs(deep.slope)  > 0.03 or slope_flag == True:
            Slope_fix() 
        if send.color_mask_subject_size[5][0] ==0 :           #黃線出障礙物的平移
            rx = 0
        send.sendContinuousValue(-400 + rx , -1000 ,0 ,1 + slope_angle ,0) 
    
    elif Straight_status == 33:  #slope fix left
        # print('Straight_status = slope fix left')
        status = "33_Slope_Fix_Left"
        if send.color_mask_subject_YMax[5][0] < 150:
            rx = 200
        elif send.color_mask_subject_YMax[5][0] > 160:
            rx = -200
        else :
            rx = 0
        if abs(deep.slope)  > 0.03 or slope_flag == True:
            Slope_fix() 
        if send.color_mask_subject_size[5][0] ==0 :           #黃線出障礙物的平移
            rx = 0
        send.sendContinuousValue(-400 + rx , 700 ,0 ,3 + slope_angle ,0) 

#--------------------Preturn Head Parameter-----------------------#
    elif Straight_status == 41:  #preturn left
        # print('Straight_status =preturn left')
        status = "41_Preturn_Left"
        send.sendContinuousValue(turn_left_x,turn_left_y,z,5,sensor)

    elif Straight_status == 42:  #preturn right
        # print('Straight_status = preturn right')
        status = "42_Preturn_Right"
        send.sendContinuousValue(turn_right_x,turn_right_y,z,-3,sensor)
#-----------------------Red Door Avoidence----------------------#
    elif Straight_status == 999:        #Y axis move right
        # print('Straight_status = imu fix and Speed')
        status = "Y axis move right"
        send.sendContinuousValue(-1100,-2400,z,0 + slope_angle,sensor) #y_move_turn x_move

    elif Straight_status == 888:        #Y axis move left
        print('Straight_status = imu fix and Speed')
        status = "Y axis move left"
        send.sendContinuousValue(-1300,1900,z,0 + slope_angle,sensor) #y_move_turn x_move


    update_values()
    
def Y_Line_avoid():
    global Y_L_Deep,Y_C_Deep,Y_R_Deep,imu_back,Y_L_flag,Y_R_flag,B_C_Deep,B_L_Deep,B_R_Deep, IMU_ok, L_line, R_line,YYDS
    if (Y_L_Deep != 24 and Y_R_Deep != 24 and send.color_mask_subject_cnts[1] == 2):    #進黃黃避障 
        YY_avoid()
    elif Yaw_wen > 0:           #左轉
        imu_back = True
        L_line = True
        while Yaw_wen <= 85: #左轉至80
            get_IMU()
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            Move(Straight_status = 23) 
            print('turn to Yellow Line LLLLLLLLL')
        while ( Y_C_Deep > YYDS):
            get_IMU()
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            Move(Straight_status = 15)      #前進
            print('close to Yellow Line LLLLLLLLLLLOOOOOOOOOOOO')
    elif Yaw_wen <= 0:          #右轉
        imu_back = True
        R_line = True
        while Yaw_wen >= -85: #右轉至85
            get_IMU()
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            Move(Straight_status = 21) 
            print('turn to Yellow Line RRRRRRRRRR')
        while (Y_C_Deep > YYDS):    #前進至靠近黃線                 
            get_IMU()
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            Move(Straight_status = 15)      #前進
            print('close to Yellow Line RRRRRRRRRROOOOOOOOOOOO')
    else:
        get_IMU()
        IMU_Angle()
        Image_Init()
        Normal_Obs_Parameter()
        Image_Info()
        pass
    get_IMU()
    if abs(Yaw_wen) > 5 :                       #回正
        while abs(Yaw_wen) > 5 :
            get_IMU()
            IMU_Angle()
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            if Yaw_wen > 0:
                Image_Init()
                Move(Straight_status = 24)
            elif Yaw_wen <= 0:
                Move(Straight_status = 22)
            print('iiiiiiiiiiiiii__________FFFFFFFFFFIIIIIIIXXXXXXX')
    else :
        pass
    Image_Init()
    Normal_Obs_Parameter()
    Image_Info()
    send.sendContinuousValue(0, 0 , 0 , 0 , 0)
    if L_line == True :
        if C_Deep > 6 :                    #直走至靠近障礙物 線在左邊
            while C_Deep > 6 :
                print('C_Deep = ',C_Deep)
                get_IMU()
                IMU_Angle()
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                Move(Straight_status = 122)
                print('GGGGGGGGOOOOOOOOOO LLLLLLLLLline')
                if C_Deep < 10 :
                    break
        # if abs(Dx) != 0 :                  #若下一步為轉頭 先右平移遠離黃線
        #     if send.color_mask_subject_YMax[1][0] > 230 :
        #         while send.color_mask_subject_YMax[1][0] > 230 :
        #             print('LLLLLLLLL MOVE')
        #             Image_Init()
        #             Normal_Obs_Parameter()
        #             Image_Info()
        #             Move(Straight_status = 32)
    elif R_line == True :
        if C_Deep > 6 :                    #直走至靠近障礙物 線在右邊
            while C_Deep > 6 :
                print('imuokkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkkk = ',IMU_ok)
                print('C_Deep = ',C_Deep)
                get_IMU()
                IMU_Angle()
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                Move(Straight_status = 122)
                print('GGGGGGGGGOOOOOOOOO RRRRRRRRRRRline')
                if C_Deep < 10 :
                    breakImage_Init()
        # if abs(Dx) != 0 :                  #若下一步為轉頭 先左平移遠離黃線
        #     if send.color_mask_subject_YMax[1][0] > 230 :
        #         while send.color_mask_subject_YMax[1][0] > 230 :
        #             print('RRRRRRRR MOVE')
        #             Image_Init()
        #             Normal_Obs_Parameter()
        #             Image_Info()
        #             Move(Straight_status = 33)
    get_IMU()
    IMU_Angle()
    Image_Init()
    Normal_Obs_Parameter()
    Image_Info()
    IMU_ok = False 
    # L_line = False
    # R_line = False

def YY_avoid():                 #黃色通道
    global Y_L_Deep,Y_C_Deep,Y_R_Deep,imu_back,Y_L_flag,Y_R_flag,B_C_Deep,B_L_Deep,B_R_Deep, IMU_ok, L_line, R_line,YYDS
    get_IMU()
    Image_Info()
    if(Yaw_wen > 5):        #左轉
        imu_back = True
        L_line = True
        while Yaw_wen <= 85:                   #向左旋轉  
            get_IMU()
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            Move(Straight_status = 23) 
            print('turn to YYY Line LLLLLLLLL')
        while ( Y_C_Deep > YYDS):                   #直走至靠近黃線
            get_IMU()
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            Move(Straight_status = 15)      #前進
            print('close to YYYYY Line LLLLLLLLLLLOOOOOOOOOOOO')
    elif(Yaw_wen < -5):     #右轉
        imu_back = True
        R_line = True
        while Yaw_wen >= -85:                   #向右旋轉
            get_IMU()
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            Move(Straight_status = 21) 
            print('turn to YYY Line RRRRRRRRRR')
        while (Y_C_Deep > YYDS):                  #直走至靠近黃線
            get_IMU()
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            Move(Straight_status = 15)      #前進
            print('close to Yellow Line RRRRRRRRRROOOOOOOOOOOO')
    elif(-5<=Yaw_wen<=5):                 #避免晃動時imu有問題 用深度值加強判斷
        if Y_L_Deep < Y_R_Deep :          #左轉
            imu_back = True
            L_line = True
            while ( Y_C_Deep > YYDS):
                if Yaw_wen <= 80:
                    while Yaw_wen <= 80:
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
                    Move(Straight_status = 15)  #前進
                    print('close to Yellow Line LLLLLLLLLLLOOOOOOOOOOOO')
        elif Y_R_Deep < Y_L_Deep:          #右轉
            imu_back = True
            R_line = True
            while (Y_C_Deep > YYDS):
                if Yaw_wen >= -80:
                    while Yaw_wen >= -80:
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
                    Move(Straight_status = 15)  #前進
                    print('close to Yellow Line RRRRRRRRRROOOOOOOOOOOO')
    get_IMU()
    if abs(Yaw_wen) > 8 :                   #回正
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
    else :
        pass
    Image_Init()
    Normal_Obs_Parameter()
    Image_Info()
    send.sendContinuousValue(0, 0 , 0 , 0 , 0)
    if L_line == True :
        if C_Deep > 6 :                    #直走至靠近障礙物 線在左邊
            while C_Deep > 6 :
                get_IMU()
                IMU_Angle()
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                if abs(Yaw_wen) < 2 :
                    Move(Straight_status = 14)
                else:
                    Move(Straight_status = 122)
                print('GGO Lline')
                if C_Deep < 10 :
                    break
        # if abs(Dx) != 0 :                  #若下一步為轉頭 先右平移遠離黃線
        #     if send.color_mask_subject_YMax[1][0] > 230 :
        #         while send.color_mask_subject_YMax[1][0] > 230 :
        #             Image_Init()
        #             Normal_Obs_Parameter()
        #             Image_Info()
        #             Move(Straight_status = 32)
    elif R_line == True : 
        if C_Deep > 6 :                    #直走至靠近障礙物 線在右邊
            while C_Deep > 6 :
                get_IMU()
                IMU_Angle()
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                if abs(Yaw_wen) < 2 :
                    Move(Straight_status = 14)
                else:
                    Move(Straight_status = 122)
                print('GGO Rline')
                if C_Deep < 10 :
                    break
        # if abs(Dx) != 0 :                  #若下一步為轉頭 先左平移遠離黃線
        #     if send.color_mask_subject_YMax[1][0] > 230 :
        #         while send.color_mask_subject_YMax[1][0] > 230 :
        #             Image_Init()
        #             Normal_Obs_Parameter()
        #             Image_Info()
        #             Move(Straight_status = 33)
    get_IMU()
    IMU_Angle()
    Image_Init()
    Normal_Obs_Parameter()
    Image_Info()
    IMU_ok = False 
    # L_line = False
    # R_line = False


def Turn_Head():
    global R_deep_sum, L_deep_sum, L_Deep, R_Deep, y_move
    Move(Straight_status = 0)
    if R_line == False and L_line == False : 
        time.sleep(1)
        send.sendHeadMotor(1,1447,100)
        send.sendHeadMotor(2,2600,100)
        send.sendHeadMotor(1,1447,100)
        send.sendHeadMotor(2,2600,100)
        send.sendHeadMotor(1,1447,100)
        send.sendHeadMotor(2,2600,100)
        time.sleep(1.3) 
        Image_Init()
        Normal_Obs_Parameter()
        R_deep_sum = Deep_sum
        send.sendHeadMotor(1,2647,100)
        send.sendHeadMotor(2,2600,100)
        send.sendHeadMotor(1,2647,100)
        send.sendHeadMotor(2,2600,100)
        send.sendHeadMotor(1,2647,100)
        send.sendHeadMotor(2,2600,100)
        time.sleep(1.8)
        Image_Init()
        Normal_Obs_Parameter()
        L_deep_sum = Deep_sum
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,2600,100)
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,2600,100)
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,2600,100)
        time.sleep(1)
        # print('R_deep_sum = ',R_deep_sum)
        # print('L_deep_sum = ',L_deep_sum)
    else :
        R_deep_sum = 0
        L_deep_sum = 0

    if (R_deep_sum > L_deep_sum) or (L_line == True):        #右轉
        Image_Init()
        Normal_Obs_Parameter()
        Image_Info()
        if ( send.color_mask_subject_YMax[2][0] < 220 ):                #靠近障礙物
            while ( send.color_mask_subject_YMax[2][0] < 220 ):
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                Move(Straight_status = 15) 
        elif ( send.color_mask_subject_YMax[2][0] > 230):                #遠離障礙物
            while ( send.color_mask_subject_YMax[2][0] > 230 ):
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                Move(Straight_status = 16) 
        print('TTTTTTTTTTTurn Headrrrrrrrr')
        get_IMU()
        if abs(Yaw_wen) < 70:                   #靠近後右旋轉至90度
            while abs(Yaw_wen) < 70:
                # print('L_line = ',L_line)
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                Move(Straight_status = 21)
        send.sendHeadMotor(1,2647,100)
        send.sendHeadMotor(2,2600,100) 
        time.sleep(1)
        Image_Init()
        Normal_Obs_Parameter()
        send.sendContinuousValue(0, 0 , 0 , 0 , 0)
        if abs(Dx) >=1  :               #直走且imu修正
            while abs(Dx) >= 1 : 
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                # print('DDDDeep = ',R_Deep)
                if (R_Deep != 24) and (R_Deep <= 14) :                  #轉頭後直走 平移修正
                    y_move = -800
                elif (R_Deep != 24) and (R_Deep > 14) :
                    y_move = 600
                if abs(Yaw_wen) > 87 :          #視步態更動
                    Move(Straight_status = 25)
                    # print(' Dx = ',Dx)
                else :
                    Move(Straight_status = 26)
                    # print(' Dx = ',Dx)
                # if C_Deep == 24 and R_Deep == 24:# MRT
                #     Move(Straight_status = 15)
                #     time.sleep(1)
                #     break
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,2600,100) 
        time.sleep(1)
        Image_Init()
        Normal_Obs_Parameter()
        #----看到黃線
        if ((deep.line_flag == True) and (send.color_mask_subject_YMin[1][0] <= 10)) or ((send.color_mask_subject_cnts[1] == 2) and (Y_L_Deep <= 7) and (Y_R_Deep <= 7)) or ((send.color_mask_subject_XMax[1][0] >= 310) and (send.color_mask_subject_XMin[1][0] <= 10) and (send.color_mask_subject_cnts[1] == 1)):
            print('YYYline___right')
            Y_Line_avoid()
        get_IMU()
        if abs(Yaw_wen) > 45:             #右轉回正
            while abs(Yaw_wen) > 45:
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                Move(Straight_status = 22)
    elif (L_deep_sum > R_deep_sum) or (R_line == True):         #左轉
        Image_Init()
        Normal_Obs_Parameter()
        Image_Info()
        if (send.color_mask_subject_YMax[2][0] < 220):                   #靠近障礙物
            while ( send.color_mask_subject_YMax[2][0] < 220 ):
                # print('send.color_mask_subject_YMax = ',send.color_mask_subject_YMax[2][0])
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                Move(Straight_status = 15) 
        elif ( send.color_mask_subject_YMax[2][0] > 230 ):                #遠離障礙物
            while ( send.color_mask_subject_YMax[2][0] > 230 ):
                # print('send.color_mask_subject_YMax = ',send.color_mask_subject_YMax[2][0])
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                Move(Straight_status = 16) 
        print('TTTTTTTTTTTurn Headlllllllllll')
        get_IMU()
        if abs(Yaw_wen) < 75:                           #靠近後轉至90度
            while abs(Yaw_wen) < 75:
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                Move(Straight_status = 23)
        send.sendHeadMotor(1,1447,100)
        send.sendHeadMotor(2,2600,100) 
        time.sleep(1)
        Image_Init()
        Normal_Obs_Parameter()
        send.sendContinuousValue(0, 0 , 0 , 0 , 0)
        if  abs(Dx) >= 1 :                          #直走且imu修正
            while  abs(Dx) >= 1 :
                # print('DDDDeep = ',L_Deep)
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                # if C_Deep >= 14 :
                #     break
                if (L_Deep != 24) and (L_Deep <= 14) :                  #轉頭後直走 平移修正
                    y_move = 200
                elif (L_Deep != 24) and (L_Deep > 14) :
                    y_move = -400
                    # if C_Deep >= 2 :
                    #     while C_Deep >= 2 :
                    #         Image_Init()
                    #         Normal_Obs_Parameter()
                    #         get_IMU()
                    #         y_move = -400
                    #         Move(Straight_status = 277)
                if abs(Yaw_wen) > 85 :          #視步態更動
                    Move(Straight_status = 27)
                    # print(' Dx = ',Dx)
                else :
                    Move(Straight_status = 28) 
                    # print(' Dx = ',Dx)
                # if C_Deep == 24 and L_Deep == 24:#MRT
                #     Move(Straight_status = 15)
                #     time.sleep(1)
                #     break
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,2600,100) 
        time.sleep(1)
        Image_Init()
        Normal_Obs_Parameter()
        #----看到黃線
        if ((deep.line_flag == True) and (send.color_mask_subject_YMin[1][0] <= 10)) or ((send.color_mask_subject_cnts[1] == 2) and (Y_L_Deep <= 7) and (Y_R_Deep <= 7)) or ((send.color_mask_subject_XMax[1][0] >= 310) and (send.color_mask_subject_XMin[1][0] <= 10) and (send.color_mask_subject_cnts[1] == 1)):
            print('YYYline___left')
            Y_Line_avoid()
        get_IMU()
        if abs(Yaw_wen) > 50:               #左轉回正
            while abs(Yaw_wen) > 50:    
                Image_Init()
                Normal_Obs_Parameter()
                get_IMU()
                Move(Straight_status = 24)

def Slope_fix():
    global slope_angle ,slope_Rcnt,slope_Lcnt
    # print('slope = ',deep.slope)
    # print('degree = ',deep.degree)
    if deep.slope > 0:          #fix to l
        if  deep.slope >= 2:
            slope_angle = 0
        elif deep.slope >= 1:
            slope_angle = 2
        elif 1 > deep.slope >= 0.3:
            slope_angle = 1
        elif 0.3 > deep.slope >= 0.2:
            slope_angle = 0
        elif 0.2 > deep.slope >= 0.15:
            slope_angle = 0
        elif 0.15 > deep.slope >= 0.1:
            slope_angle = 0
        elif 0.1 > deep.slope >= 0.05:
            slope_angle = 0
        elif 0.05 > deep.slope >= 0.03:
            slope_angle = 0
        elif 0.03 > deep.slope >= 0:
            slope_angle = 0
            slope_Lcnt  += 1
    elif deep.slope <= 0:       #fix to r
        if  deep.slope <= -2:
            slope_angle = 0
        elif -1 >= deep.slope:
            slope_angle = -5
        elif -0.3 >= deep.slope > -1:
            slope_angle = -4
        elif -0.2 >= deep.slope > -0.3:
            slope_angle = -3
        elif -0.15 >= deep.slope > -0.2:
            slope_angle = -2
        elif -0.1 >= deep.slope > -0.15:
            slope_angle = -1
        elif -0.05 >= deep.slope > -0.1:
            slope_angle = 0
        elif -0.03 >= deep.slope > -0.05:
            slope_angle = 0
        elif 0 >= deep.slope > -0.03:
            slope_angle = 0
            slope_Rcnt  += 1
    if send.color_mask_subject_size[5][0] ==0 :
        slope_angle = 0

    # print( 'slope_angle = ' + str(slope_angle))
    return slope_angle

def IMU_Yaw_ini():
    global  Yaw_wen
    Yaw_wen = 0
    # update_values()
def get_IMU():
    global Yaw_wen
    Yaw_wen = send.imu_value_Yaw
    print('Yaw = ' + str(Yaw_wen))
    # update_values()
    return Yaw_wen

def IMU_Angle():
    global imu_angle
    if Yaw_wen > 0:         #fix to r
        if Yaw_wen >= 90:
            imu_angle = -9
        elif 90 > Yaw_wen >= 60:
            imu_angle = -8
        elif 60 > Yaw_wen >= 45:
            imu_angle = -7
        elif 45 > Yaw_wen >= 20:
            imu_angle = -6
        elif 20 > Yaw_wen >= 10:
            imu_angle = -5
        elif 10 > Yaw_wen >= 5:
            imu_angle = -4
        elif 5 > Yaw_wen >= 2:
            imu_angle = -3
        elif 2 > Yaw_wen >= 0:
            imu_angle = 0
    elif Yaw_wen <= 0:      #fix to l
        if -90 >= Yaw_wen:
            imu_angle = 9
        elif -60 >= Yaw_wen > -90:
            imu_angle = 8
        elif -45 >= Yaw_wen > -60:
            imu_angle = 7
        elif -20 >= Yaw_wen > -45:
            imu_angle = 6
        elif -10 >= Yaw_wen > -20:
            imu_angle = 5
        elif -5 >= Yaw_wen > -10:
            imu_angle = 4
        elif -2 >= Yaw_wen > -5:
            imu_angle = 3
        elif 0 >= Yaw_wen > -2:
            imu_angle = 0
    # print( 'imu_angle = ' + str(imu_angle))
    # update_values()
    return imu_angle

def Straight_Speed():
    global Goal_speed 
    if Dy ==24:
        Goal_speed = 3300
    elif 20 <= Dy < 24:
        Goal_speed = 3000
    elif 16 <= Dy < 20:
        Goal_speed = 2500
    elif 14 <= Dy < 16:
        Goal_speed = 2200
    elif 12 <= Dy < 14:
        Goal_speed = 1600
    elif 8 <= Dy < 12:
        Goal_speed = 900
    elif 6 <= Dy < 8:
        Goal_speed = 400
    elif 3 <= Dy < 6:
        Goal_speed = 300
    elif 0 <= Dy < 3:
        Goal_speed = 0
    # print( 'Goal_speed = ' + str(Goal_speed))
    # update_values()
    return Goal_speed

def Turn_Angle(Turn_angle_status):
    global Angle
    if Turn_angle_status == 0:      #R
        print('turn right')
        if 17 > Dx >= 12:
            Angle = -9
        elif 12 > Dx >= 8:
            Angle = -9
        elif 8 > Dx >= 6:
            Angle = -7
        elif 6 > Dx >= 4:
            Angle = -6
        elif 4 > Dx >= 2:
            Angle = -4
        elif 2 > Dx >= 0:
            Angle = 0
    elif Turn_angle_status == 1:    #L
        print('turn left')
        if -12 >= Dx > -17:
            Angle = 9
        elif -8 >= Dx > -12:
            Angle = 8
        elif -6 >= Dx > -8:
            Angle = 7
        elif -4 >= Dx > -6:
            Angle = 5
        elif -3 >= Dx > -4:
            Angle = 4
        elif -2 >= Dx > -3:
            Angle = 3
        elif 0 >= Dx > -2:
            Angle = 0
    else: 
        Angle = 0
    # print( 'Angle = ' + str(Angle))
    # update_values()
    return Angle

def Crawl():
    global Straight_status,crawl_cnt,C_Deep,L_Deep,R_Deep,CRMax,CRMin,B_min,B_max,B_left,B_right,slope_Rcnt,slope_Lcnt
    slope_Rcnt = 0
    slope_Lcnt = 0
    if send.color_mask_subject_YMax[5][0] < CRMin or send.color_mask_subject_YMax[5][0] > CRMax or CRMin <= send.color_mask_subject_YMax[5][0] <= CRMax:
        while send.color_mask_subject_YMax[5][0] < CRMin or send.color_mask_subject_YMax[5][0] > CRMax or CRMin <= send.color_mask_subject_YMax[5][0] <= CRMax:
            print('min = ',B_min)
            print('max = ',B_max)
            print('Left = ',B_left)
            print('Right = ',B_right)

            if(send.color_mask_subject_YMax[5][0] < CRMin):            #前進修正
                Slope_fix()
                Move(Straight_status = 15)
                print('crawlllllll forwardddddddddd')
            elif(send.color_mask_subject_YMax[5][0] > CRMax):          #後退修正
                Slope_fix()
                Move(Straight_status = 16)
                print('crawlllllll backkkkkkkk')
            elif CRMin <= send.color_mask_subject_YMax[5][0] <= CRMax:    #爬
                if abs(deep.slope) > 0.03:                                #不平行紅門時修斜率
                    while abs(deep.slope) > 0.03:
                        Slope_fix()
                        Move(Straight_status = 31)
                        Image_Info()
                        print('rcnt = ',slope_Rcnt)
                        print('lcnt = ',slope_Lcnt)
                        if slope_Rcnt > 400 or slope_Lcnt > 400:
                            break
                    # slope_flag = False
                    # slope_Lcnt = 0
                    # slope_Rcnt = 0
                print('CCCCCCCCCCCCCCCCRWAL')
                send.sendContinuousValue(0, 0 , 0 , 0 , 0) 
                time.sleep(1)
                # send.sendBodyAuto(0,0,0,0,1,0)
                time.sleep(2)
                #send.sendBodySector(299)    #基礎站姿29！！！！！！！！！！！！！！！！！！
                time.sleep(4)
                time.sleep(20)
                # send.sendBodySector(123)
                # time.sleep(9)
                # time.sleep(0.5)
                # while crawl_cnt < 3:                #避免門下建模有問題 固定爬三次才抬頭
                #     send.sendBodySector(456)
                #     time.sleep(2.8)
                #     # time.sleep(0.3)
                #     crawl_cnt += 1
                # send.color_mask_subject_YMax[1][0] = 0
                # send.color_mask_subject_YMax[2][0] = 0
                # send.sendHeadMotor(1,2048,100)
                # send.sendHeadMotor(2,2500,100)
                # time.sleep(1)
                # while crawl_cnt < 8:                   #邊爬邊判斷是否離障礙物太近
                #     Image_Init()
                #     Normal_Obs_Parameter()
                #     time.sleep(0.1)
                #     print('send.color_mask_subject_YMax[2][0] = ',send.color_mask_subject_YMax[2][0])
                #     print('send.color_mask_subject_YMax[1][0] = ',send.color_mask_subject_YMax[1][0])
                #     if (send.color_mask_subject_YMax[2][0] >= 85 and send.color_mask_subject_size[2][0] > 5000) or (send.color_mask_subject_YMax[1][0] >= 60 and send.color_mask_subject_size[1][0] > 5000):
                #         break
                #     else:
                #         send.sendBodySector(456)
                #         time.sleep(2.8)
                #         #time.sleep(0.1)
                #         crawl_cnt += 1
                # send.sendBodySector(1113)
                # time.sleep(14.4)
                # time.sleep(1.5)
                # send.sendBodySector(299)    #基礎站姿29！！！！！！！！！！！！！！！！！！
                # time.sleep(1.5)
                send.sendHeadMotor(1,2048,100)
                send.sendHeadMotor(2,2600,100)
                time.sleep(0.5)
                # send.sendBodyAuto(0,0,0,0,1,0)
                time.sleep(1)
                break
def GuoLe1():
    global L_line, R_line, function
    Image_Init()
    Normal_Obs_Parameter()
    L_line = True
    R_line = False
    function ="過了過了"
    update_values()
    # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    Turn_Head()
    L_line = False
    R_line = False
    function ="還沒拉！幹"
    update_values()
def GuoLe2():
    global L_line, R_line, function
    Image_Init()
    Normal_Obs_Parameter()
    L_line = False
    R_line = True
    function ="過了過了"
    update_values()
    # print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
    Turn_Head()
    L_line = False
    R_line = False
    function ="還沒拉！幹"
    update_values()
def red_door_avoidance():
    global slope_flag, slope_Lcnt ,slope_Rcnt, x_move, y_move_turn
    if abs(deep.slope) > 0.4 and (slope_flag == True):                     #不平行紅門時修斜率
        while abs(deep.slope) > 0.4:
            Slope_fix()
            Move(Straight_status = 31)
            Image_Init()
            Normal_Obs_Parameter()
            if slope_Rcnt > 400 or slope_Lcnt > 400:
                break
        slope_flag = False
        slope_Lcnt = 0
        slope_Rcnt = 0
    Image_Init()
    Normal_Obs_Parameter()
    Image_Info()
    if ( send.color_mask_subject_YMax[5][0] < 165 ):                #靠近障礙物
        while ( send.color_mask_subject_YMax[2][0] < 165 ):
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            update_values()
            Move(Straight_status = 15) 
    elif ( send.color_mask_subject_YMax[5][0] > 165):                #遠離障礙物
        while ( send.color_mask_subject_YMax[2][0] > 165 ):
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            update_values()
            Move(Straight_status = 16) 
    if(0 <= send.color_mask_subject_X [5][0] < 150): #紅門在左邊
        while abs(Dx) > 9:
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            update_values()
            # if ( send.color_mask_subject_YMax[2][0] < 160 ):               #平移修正 #靠近障礙物
            #     x_move = 500 
            # elif ( send.color_mask_subject_YMax[2][0] > 170):                #遠離障礙物
            #     x_move = -500 
            # elif (160 < send.color_mask_subject_YMax[2][0] > 170):
            #     x_move = 0
            if abs(deep.slope) > 0.4 and send.color_mask_subject_XMax[5][0] > 90:                     #不平行時修斜率
                while abs(deep.slope) > 0.4 and send.color_mask_subject_XMax[5][0] > 90:
                    Slope_fix()
                    Image_Info()
                    Move(Straight_status = 999) #右平移
                    # if slope_Rcnt > 400 or slope_Lcnt > 400:
                    #     break
            # if (B_L_Deep < B_R_Deep) and abs(deep.slope) <= 0 and B_R_Deep < 20:
            #     y_move_turn = 1
            # elif (B_L_Deep > B_R_Deep) and abs(deep.slope) <= 0 :
            #     y_move_turn = -1
            # else:
            #     y_move_turn = 0
            else:
                Image_Init()
                Normal_Obs_Parameter()
                # Image_Info()
                update_values()
                Move(Straight_status = 999) #右平移
        slope_Lcnt = 0
        slope_Rcnt = 0
    elif(160 < send.color_mask_subject_X [5][0] <= 320): #紅門在右邊
        while abs(Dx) > 9:
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            update_values()
            x_move = 0
            if ( send.color_mask_subject_YMax[2][0] < 160 ):               #平移修正 #靠近障礙物
                x_move = 500 
            elif ( send.color_mask_subject_YMax[2][0] > 170):                #遠離障礙物
                x_move = -500 
            elif (160 < send.color_mask_subject_YMax[2][0] > 170):
                x_move = 0
            if abs(deep.slope) > 0.03 and send.color_mask_subject_XMin[5][0] < 230:                     #不平行時修斜率
                while abs(deep.slope) > 0.03:
                    Slope_fix()
                    Image_Info()
                    if slope_Rcnt > 400 or slope_Lcnt > 400:
                        break
            # if (B_L_Deep < B_R_Deep) and abs(deep.slope) <= 0 :
            #     y_move_turn = 1
            # elif (B_L_Deep > B_R_Deep) and abs(deep.slope) <= 0 and B_L_Deep < 20:
            #     y_move_turn = -1
            # else:
            #     y_move_turn = 0
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            Move(Straight_status = 888) #左平移
        slope_flag = False
        slope_Lcnt = 0
        slope_Rcnt = 0
    elif(150 <= send.color_mask_subject_X [5][0] <= 160): #紅門在中間
        Red_Turn_Head()

def Red_Turn_Head():
    global R_deep_sum, L_deep_sum, L_Deep, R_Deep, slope_flag, slope_Lcnt, slope_Rcnt, x_move
    Move(Straight_status = 0)
    if R_line == False and L_line == False : 
        time.sleep(1)
        send.sendHeadMotor(1,1447,100)
        send.sendHeadMotor(2,2600,100)
        send.sendHeadMotor(1,1447,100)
        send.sendHeadMotor(2,2600,100)
        send.sendHeadMotor(1,1447,100)
        send.sendHeadMotor(2,2600,100)
        time.sleep(1.3) 
        Image_Init()
        Normal_Obs_Parameter()
        R_deep_sum = Deep_sum
        send.sendHeadMotor(1,2647,100)
        send.sendHeadMotor(2,2600,100)
        send.sendHeadMotor(1,2647,100)
        send.sendHeadMotor(2,2600,100)
        send.sendHeadMotor(1,2647,100)
        send.sendHeadMotor(2,2600,100)
        time.sleep(1.8)
        Image_Init()
        Normal_Obs_Parameter()
        L_deep_sum = Deep_sum
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,2600,100)
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,2600,100)
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,2600,100)
        time.sleep(1)
    else :
        R_deep_sum = 0
        L_deep_sum = 0
    if (R_deep_sum > L_deep_sum) or (L_line == True):        #右平移
        while abs(Dx) > 9:
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            update_values()
            if ( send.color_mask_subject_YMax[2][0] < 160 ):               #平移修正 #靠近障礙物
                x_move = 500 
            elif ( send.color_mask_subject_YMax[2][0] > 170):                #遠離障礙物
                x_move = -500 
            elif (160 < send.color_mask_subject_YMax[2][0] > 170):
                x_move = 0
            if abs(deep.slope) > 0.03 and send.color_mask_subject_XMax[5][0]  > 90:                     #不平行時修斜率
                while abs(deep.slope) > 0.03:
                    Slope_fix()
                    Image_Info()
                    if slope_Rcnt > 400 or slope_Lcnt > 400:
                        break
            if (B_L_Deep < B_R_Deep) and abs(deep.slope) <= 0 and B_R_Deep < 20:
                y_move_turn = 1
            elif (B_L_Deep > B_R_Deep) and abs(deep.slope) <= 0 :
                y_move_turn = -1
            else:
                y_move_turn = 0
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            Move(Straight_status = 999) #右平移
        slope_flag = False
        slope_Lcnt = 0
        slope_Rcnt = 0
        L_line == False
    elif (L_deep_sum > R_deep_sum) or (R_line == True):         #左平移
        while abs(Dx) > 9:
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            update_values()
            if ( send.color_mask_subject_YMax[2][0] < 160 ):               #平移修正 #靠近障礙物
                x_move = 500 
            elif ( send.color_mask_subject_YMax[2][0] > 170):                #遠離障礙物
                x_move = -500 
            elif (160 < send.color_mask_subject_YMax[2][0] > 170):
                x_move = 0
            if abs(deep.slope) > 0.03 and send.color_mask_subject_XMin[5][0] < 230:                     #不平行時修斜率
                while abs(deep.slope) > 0.03:
                    Slope_fix()
                    Image_Info()
                    if slope_Rcnt > 400 or slope_Lcnt > 400:
                        break
            if (B_L_Deep < B_R_Deep) and abs(deep.slope) <= 0 :
                y_move_turn = 1
            elif (B_L_Deep > B_R_Deep) and abs(deep.slope) <= 0 and B_L_Deep < 20:
                y_move_turn = -1
            else:
                y_move_turn = 0
            Image_Init()
            Normal_Obs_Parameter()
            Image_Info()
            Move(Straight_status = 888) #左平移
        slope_flag = False
        slope_Lcnt = 0
        slope_Rcnt = 0
        R_line == False

if __name__ == '__main__':
    try:
        print("try main")
        deep = deep_calculate()
        send = Sendmessage()
        while not rospy.is_shutdown():
            # print('YMax = ',send.color_mask_subject_YMax[2][0])
            # print('Dx = ',Dx)
            if send.is_start == True:
                #==============================image===============================
                # Focus_Matrix = [6, 7, 8, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 8, 8, 7, 6]
                Focus_Matrix = [0, 0, 0, 0, 4, 4, 5, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 5, 4, 4, 0, 0, 0, 0]#0, 1, 2, 4, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 9, 9, 8, 8, 7, 7, 7, 6, 6, 6, 6, 5, 5, 4, 2, 1, 0
                YYDS =  2   #Y_Line_Deep 2  4
                CRMax = 65 #R_Max       65 55
                CRMin = 55 #R_MIn       55 45
                #sector:1218 1420 5 6 
                Image_Init()
                Normal_Obs_Parameter()
                Image_Info()
                #=============================strategy=============================
                if walking == False:                        #指撥後初始動作
                    PreTurn_L = False
                    #PreTurn_L = True
                    PreTurn_R = False
                    #PreTurn_R = True
                    send.sendHeadMotor(1,2048,100)
                    send.sendHeadMotor(2,2680,100)
                    #send.sendBodySector(15)     #收手 小白
                    #send.sendBodySector(16)     #收手 小黑
                    #send.sendBodySector(1812)   #長腳
                    time.sleep(0.5)
                    send.sendBodyAuto(0,0,0,0,1,0)
                    time.sleep(0.5) 
                walking = True
                if PreTurn_L == True :                      #指定初始向左旋轉
                    get_IMU()
                    if abs(Yaw_wen) < 30:
                        while abs(Yaw_wen) < 30:
                            get_IMU()
                            Move(Straight_status = 23)
                            # PreTurn_L = False
                    PreTurn_L = False
                elif PreTurn_R == True :                        #指定初始向右旋轉 70
                    get_IMU()
                    if abs(Yaw_wen) < 45:
                        while abs(Yaw_wen) < 45:
                            get_IMU()
                            Move(Straight_status = 21)
                            # PreTurn_R = False
                    PreTurn_R = False
                # if (send.color_mask_subject_cnts[1] == 2) and (send.color_mask_subject_YMax[1][0] >= 220) and (send.color_mask_subject_YMax[1][1] >= 220):#and (send.color_mask_subject_YMin[1][0] < 150)
                #     if B_C_Deep >=9:
                #         while B_C_Deep >=9:
                #             if Y_XMax_two > 260 :
                #                 while Y_XMax_two > 260 :
                #                     Move(Straight_status = 14)
                #                     if  Y_XMax_two < 260 or  send.color_mask_subject_cnts[1] == 1:
                #                         break
                #             if (abs(Yaw_wen) > 5) :             
                #                 while abs(Yaw_wen) > 5:
                #                     print("HHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHHH")
                #                     Image_Init()
                #                     Normal_Obs_Parameter()
                #                     get_IMU()
                #                     Move(Straight_status = 12)
                #             if B_C_Deep >= 9 :
                #                 Move(Straight_status = 14)
                #             else :
                #                 break
                #Dx = 0
                    #if red_flag == True:                    #若有紅門
                #     print('In Reddoor')
                #     if First_Reddoor == False :
                #         First_Reddoor = True
                #         send.sendHeadMotor(1,2048,100)
                #         send.sendHeadMotor(2,1800,100)
                #         time.sleep(0.5)
                #         send.sendContinuousValue(0, 0 , 0 , 0 , 0)
                   
                #     elif First_Reddoor == True :

                #         print('YYYYYYYYYYYYYYYMMMMMMMMMIIIIIIIIINNNNNNNNN = ',send.color_mask_subject_YMax[5][0])
                #         if send.color_mask_subject_YMax[5][0] > 235:
                #             while send.color_mask_subject_YMax[5][0] > 235:
                #                 Image_Init()
                #                 Normal_Obs_Parameter()
                #                 Image_Info()
                #                 Move(Straight_status = 16)

                #         print('slope = ',deep.slope)
                #         if abs(deep.slope) > 0.03 and (slope_flag == True):                     #不平行紅門時修斜率
                #             while abs(deep.slope) > 0.03 or slope_flag == True:
                #                 print('YYYYYYYYYYYYYYYMMMMMMMMMIIIIIIIIINNNNNNNNN = ',send.color_mask_subject_YMax[5][0])
                #                 Slope_fix()
                #                 Move(Straight_status = 31)
                #                 Image_Info()
                #                 if slope_Rcnt > 400 or slope_Lcnt > 400:
                #                     break
                #             slope_flag = False
                #             slope_Lcnt = 0
                #             slope_Rcnt = 0
                #             send.sendHeadMotor(1,2048,100)
                #             send.sendHeadMotor(2,1800,100)
                #             time.sleep(0.5)
                #         else :
                #             if (send.color_mask_subject_YMax[5][0] < 150) and (redoor_dis == False) :     #前後距離修正（值越大離門越近） 55/65   離紅門太遠時前進
                #                 Slope_fix()
                #                 Move(Straight_status = 15)
                #                 pass
                #             elif (send.color_mask_subject_YMax[5][0] > 160) and (redoor_dis == False) :   #前後距離修正（值越大離門越近） 55/65   離紅門太近時候退
                #                 Slope_fix()
                #                 Move(Straight_status = 16)
                #                 pass
                #             else :                          #判斷須左移或右移 都不需則進爬
                #                 redoor_dis = True
                #                 if R_min < 2 and R_max > 315 :
                #                     print('red center')
                #                     print('BBBBBBBRRRRRRRRRRRRFFFFFFLLLLLLLAAAAAAAGGGGG = ',BR_flag)
                #                     print('BBBLLLLLLLLLLLLLLLLFFFFFFFFFFFFFFFF = ',BL_flag)
                #                     if (B_max == 0 and B_min == 0 and B_left <= 35 and BR_flag == True) or (B_max == 0 and B_min == 0 and B_right > 275 and BL_flag == True) or (B_max == 0 and B_min == 0 and B_right == 0 and B_left == 0):
                #                         if abs(deep.slope)  > 0.03 or slope_flag == True:
                #                             while abs(deep.slope) > 0.03 or slope_flag == True:
                #                                 print('min = ',B_min)
                #                                 print('max = ',B_max)
                #                                 print('Left = ',B_left)
                #                                 print('Right = ',B_right)
                #                                 print('ddddddddddddddddddddddd = ',send.color_mask_subject_YMax[5][0])
                #                                 Slope_fix()
                #                                 Move(Straight_status = 31)
                #                                 print('crawl22222222222222222222222222222222')
                #                                 Image_Info()
                #                                 if slope_Rcnt > 400 or slope_Lcnt > 400:
                #                                     break
                #                         else :
                #                             Crawl()
                #                     elif (B_min < 2 and B_max > 20):
                #                         print('move R 11111')
                #                         BR_flag = True
                #                         BL_flag = False
                #                         Move(Straight_status = 32)
                #                     elif (B_max > 315 and B_min < 300):
                #                         print('move L 11111')
                #                         BL_flag = True
                #                         BR_flag = False
                #                         Move(Straight_status = 33)
                #                     else :
                #                         if BR_flag == True:
                #                             Move(Straight_status = 32)
                #                             print('BBBBBBBBBBBBBBRRRRRRRRRRR =',BR_flag)
                #                         elif BL_flag == True:
                #                             Move(Straight_status = 33)
                #                             print('BBBBBBBBBBLLLLLLLLLLLLLLLLLLLL = ',BL_flag)
                #                 elif R_min < 2 and R_max < 315 : 
                #                     print('move L')
                #                     Move(Straight_status = 33)
                #                 elif R_min > 2 and R_max > 315 : 
                #                     print('move R')
                #                     Move(Straight_status = 32)
                #else :
                get_IMU()
                #print('xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx = ',Y_left)
                if Dy < 24:
                    # if R_line == True or L_line == True:
                    #     while R_line ==True or L_line == True:
                    #         Image_Init()
                    #         Normal_Obs_Parameter()
                    #         YL_Deep_sum = Y_Deep_sum1
                    #         YR_Deep_sum = Y_Deep_sum2
                    #         print('RRRRRRR',YR_Deep_sum)
                    #         print('LLLLLLL',YL_Deep_sum)
                    Image_Init()
                    Normal_Obs_Parameter()
                    YL_Deep_sum = Y_Deep_sum1
                    YR_Deep_sum = Y_Deep_sum2
                    if R_line == True :
                        if YL_Deep_sum > YR_Deep_sum :
                            R_line = True
                        elif (YL_Deep_sum < YR_Deep_sum) or (YR_Deep_sum > 350) :
                            R_line = False
                    elif L_line == True :
                        if YL_Deep_sum < YR_Deep_sum :
                            L_line = True
                        elif (YL_Deep_sum > YR_Deep_sum) or (YL_Deep_sum > 350) :
                            L_line = False
                    if send.color_mask_subject_YMax[5][0] >= 100:
                        print("RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR")
                        red_door_avoidance()
                    
                            # if YR_Deep_sum > 100:#待更改
                            #     R_line = True
                            # else :
                            #     R_line = False
                    if 13 > Dx > 9 :        #turn right
                        print('right avoid')
                        Straight_Speed()
                        #if ((deep.line_flag == True) and (send.color_mask_subject_YMin[1][0] <= 10)) or ((send.color_mask_subject_cnts[1] == 2) and (Y_L_Deep <= 7) and (Y_R_Deep <= 7)) or ((send.color_mask_subject_XMax[1][0] >= 310) and (send.color_mask_subject_XMin[1][0] <= 10) and (send.color_mask_subject_cnts[1] == 1)):
                            #Y_Line_avoid()      #黃線策略
                        #-------------紅門避障-------------
                        # if send.color_mask_subject_YMax[5][0] >= 100:
                        #     print("RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR")
                        #     red_door_avoidance()
                        # else :
                        #-------------紅門避障-------------
                        if (send.color_mask_subject_YMax[2][0] >= 190) and ( abs(Yaw_wen) <= 5 ) and imu_back == False and abs(Dx) > 3 and C_Deep != 24:        #離障礙物太近-->後退
                            while (send.color_mask_subject_YMax[2][0] >= 165):
                                Image_Init()
                                Normal_Obs_Parameter()
                                Image_Info()
                                Move(Straight_status = 16) 
                                print('imu fix back')
                                if send.color_mask_subject_YMax[2][0] >0 :
                                    break
                            imu_back = True
                        if ( abs(Yaw_wen) > 3 and IMU_ok == False ) and Dx >= 7 :       #IMU修正
                            get_IMU()
                            IMU_Angle()
                            Move(Straight_status = 12)
                            Image_Init()
                            Normal_Obs_Parameter()
                            Image_Info()
                        # elif Y_Dy < 2 and B_LC_Deep < 11 and B_C_Deep < 11 and B_R_Deep < 11 :
                        #     if B_Deep_sum1 > B_Deep_sum2 : 
                        #         GuoLe1()
                        #     elif  B_Deep_sum1 < B_Deep_sum2:
                        #         GuoLe2()
                        #     else : 
                        #         break
                        else:
                            Turn_Angle(Turn_angle_status = 0)
                            Move(Straight_status = 11)

                        if abs(Yaw_wen) <= 3 :
                            IMU_ok = True
                    elif -9 > Dx > -13 :     #turn left
                        print('left avoid')
                        Straight_Speed()
                        #if ((deep.line_flag == True) and (send.color_mask_subject_YMin[1][0] <= 10)) or ((send.color_mask_subject_cnts[1] == 2) and (Y_L_Deep <= 7) and (Y_R_Deep <= 7)) or ((send.color_mask_subject_XMax[1][0] >= 310) and (send.color_mask_subject_XMin[1][0] <= 10) and (send.color_mask_subject_cnts[1] == 1)):
                            #Y_Line_avoid()      #黃線策略
                        # if send.color_mask_subject_YMax[5][0] >= 100:
                        #     red_door_avoidance()
                        # else :
                        if (send.color_mask_subject_YMax[2][0] >= 190) and ( abs(Yaw_wen) <= 5 ) and imu_back == False and abs(Dx) > 3 and C_Deep != 24:        #離障礙物太近-->後退
                            while (send.color_mask_subject_YMax[2][0] >= 165):
                                Image_Init()
                                Normal_Obs_Parameter()
                                Image_Info()
                                Move(Straight_status = 16) 
                                print('imu fix back')
                                if send.color_mask_subject_YMax[2][0] > 0 :
                                    break
                            imu_back = True
                        if ( abs(Yaw_wen) > 3 and IMU_ok == False ) and Dx <= -7 :      #IMU修正
                            get_IMU()
                            IMU_Angle()
                            Move(Straight_status = 12)
                            Image_Init()
                            Normal_Obs_Parameter()
                            Image_Info()
                    # elif Y_Dy < 2 and B_LC_Deep < 11 and B_C_Deep < 11 and B_R_Deep < 11 :
                    #     if B_Deep_sum1 > B_Deep_sum2 : 
                    #         GuoLe1()
                    #     elif  B_Deep_sum1 < B_Deep_sum2:
                    #         GuoLe2()
                    #     else : 
                    #         break
                        else:
                            Turn_Angle(Turn_angle_status = 1)
                            Move(Straight_status = 11)

                        if abs(Yaw_wen) <= 3 :
                            IMU_ok = True
                    elif (Dx < 17 and Dx >= 13) or (Dx <= -13 and Dx > -17) :
                        IMU_Angle()
                        #if ((deep.line_flag == True) and (send.color_mask_subject_YMin[1][0] <= 10)) or ((send.color_mask_subject_cnts[1] == 2) and (Y_L_Deep <= 7) and (Y_R_Deep <= 7)) or ((send.color_mask_subject_XMax[1][0] >= 310) and (send.color_mask_subject_XMin[1][0] <= 10) and (send.color_mask_subject_cnts[1] == 1)):
                            #Y_Line_avoid()      #黃線策略
                        if (send.color_mask_subject_YMax[2][0] >= 200) and ( abs(Yaw_wen) <= 5 ) and imu_back == False and abs(Dx) > 3 and C_Deep != 24:        #離障礙物太近-->後退
                            while (send.color_mask_subject_YMax[2][0] >= 195):
                                Image_Init()
                                Normal_Obs_Parameter()
                                Image_Info()
                                Move(Straight_status = 16) 
                                print('imu fix back')
                                if send.color_mask_subject_YMax[2][0] >0 :
                                    break
                            imu_back = True
                        if ( abs(Yaw_wen) > 3 and IMU_ok == False ) :                   #IMU修正
                            while ( abs(Yaw_wen) > 3 or IMU_ok == False) :
                                IMU_Angle()
                                get_IMU()
                                Move(Straight_status = 12)
                                Image_Init()
                                Normal_Obs_Parameter()
                                Image_Info()

                                if abs(Yaw_wen) < 5:        #轉頭策略
                                    # if Y_Dy < 2 and B_LC_Deep < 11 and B_C_Deep < 11 and B_R_Deep < 11 :
                                    #     if B_Deep_sum1 > B_Deep_sum2 : f ( abs(Yaw_wen) > 3 and IMU_ok == False ) and Dx >= 7 :       #IMU修正
                                    get_IMU
                                    #         GuoLe1()
                                    #     elif  B_Deep_sum1 < B_Deep_sum2:
                                    #         GuoLe2()
                                    #     else : 
                                    #         break
                                    Image_Init()
                                    Normal_Obs_Parameter()
                                    Image_Info()

                                    # if (abs(Dx) < 15):
                                    #     break
                                    if ( B_L_Deep < 12 ) and ( B_R_Deep < 12 ) and ( B_C_Deep < 12 ):
                                        Turn_Head()
                                        IMU_ok = True
                                        break
                                    elif (B_L_Deep >= B_R_Deep) :#B_L_Deep >= B_R_Deep #l no obs
                                        if Y_Dy < 12 and Y_Deep_sum1 < Y_Deep_sum2 : #y at left
                                            if B_LC_Deep < 12 :
                                                while(abs(Yaw_wen) < 30):
                                                    get_IMU()
                                                    Move(Straight_status = 21)
                                                    print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
                                                IMU_ok = True
                                            else :
                                                while(abs(Yaw_wen) < 30):
                                                    get_IMU()
                                                    Move(Straight_status = 23)
                                                    print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
                                                IMU_ok = True
                                            # if Dx > 0 :
                                            #     Move(Straight_status = 11)
                                            # elif Dx < 0 :
                                            break
                                        else :
                                            while(abs(Yaw_wen) < 30):
                                                    get_IMU()
                                                    Move(Straight_status = 23)
                                                    print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
                                            IMU_ok = True
                                            break
                                    elif (B_L_Deep < B_R_Deep) : #B_L_Deep < B_R_Deep #r no obs
                                        if Y_Dy < 12 and Y_Deep_sum1 > Y_Deep_sum2 : #y at right
                                            if B_RC_Deep < 12 :
                                                while(abs(Yaw_wen) < 30):
                                                    get_IMU()
                                                    Move(Straight_status = 23)
                                                    print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
                                                IMU_ok = True
                                            else :
                                                while(abs(Yaw_wen) < 30):
                                                    get_IMU()
                                                    Move(Straight_status = 21)
                                                    print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
                                                IMU_ok = True
                                            # if Dx > 0 :
                                            #     Move(Straight_status = 11)
                                            # elif Dx < 0 :
                                            break
                                        else :
                                            while(abs(Yaw_wen) < 30):
                                                    get_IMU()
                                                    Move(Straight_status = 21)
                                                    print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
                                            IMU_ok = True
                                            break

                                else:
                                    print('IMU NOT OK')
                                    pass
                        # elif (abs(Yaw_wen) < 3 and IMU_ok == True):
                        #     print('OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO')
                        #     IMU_Angle()
                        #     get_IMU()
                        #     Move(Straight_status = 12)
                        #     Image_Init()
                        #     Normal_Obs_Parameter()
                        #     Image_Info()
                        #     if abs(Yaw_wen) < 3: 
                        #         if ( B_L_Deep < 10 ) and ( B_R_Deep < 10 ) and ( B_C_Deep < 10 ):
                        #             Turn_Head()
                        #             IMU_ok = True
                        #             break
                        #         else :
                        #             if Dx > 0 :
                        #                 Move(Straight_status = 11)
                        #             elif Dx < 0 :
                        #                 Move(Straight_status = 11)
                            # else:
                            #     print('IMU NOT OK')
                            #     pass


                    elif (9 >= Dx >= -9) or abs(Dx) >= 17:                  #最高速直走
                        # send.sendContinuousValue(0,0,0,0,0)
                        IMU_Angle()
                        get_IMU()
                        Image_Init()
                        Normal_Obs_Parameter()
                        Image_Info()
                        print('no avoid')
                        Move(Straight_status = 14)
                        if Dx == 0 :
                            IMU_ok = False
                            imu_back = False
                    else:
                        pass
                elif Dy == 24:
                    # send.sendContinuousValue(0,0,0,0,0)
                    print('go straight')
                    Straight_Speed()
                    Move(Straight_status = 14)
                print('IMU_ok ====== ' + str(IMU_ok))
            if send.is_start == False:
                print("stop")
                if walking == True:
                    send.sendContinuousValue(0,0,0,0,0)
                    time.sleep(1.5) 
                    send.sendBodyAuto(0,0,0,0,1,0)
                    time.sleep(1)
                    #send.sendBodySector(1218)    #基礎站姿29！！！！！！！！！！！！！！！！！！
                    walking = False
                IMU_Yaw_ini()
            # print('walking ====== ' + str(walking))
            Image_Init()
            Slope_fix()
            Normal_Obs_Parameter()
            # update_values()
    except rospy.ROSInterruptException:
        pass
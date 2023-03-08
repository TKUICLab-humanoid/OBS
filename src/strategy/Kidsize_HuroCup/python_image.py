#!/usr/bin/env python
#coding=utf-8
from select import select
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
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import time
import curses
from std_msgs.msg import String

# YYDS = 0
#----walking & else----
TurnHead_Flag = False
status = ""
Turn_angle_status = Turn_status = Straight_status = Goal_speed = 0
Yaw_wen = imu_angle = slope_angle = y_move = 0
walking = IMU_ok = False
R_line = L_line = imu_back = False
#----DeepMatrix----
Filter_Matrix = []
Dy1 = Y_Dy = B_Dy = 24
Dx1 = 0
WR = WL = 0
Xb = Xc = Xc_count = Xc_num = 0
Deep_sum = R_deep_sum = L_deep_sum = L_Deep = R_Deep = C_Deep = 0
#----BLUE_DeepMatrix---
B_Deep_sum = B_L_Deep = B_R_Deep = B_C_Deep = 0
#----Y_line DeepMatrix---
Y_Deep_sum = Y_Deep_sum1 = Y_Deep_sum2 = YR_Deep_sum = YL_Deep_sum = Y_L_Deep = Y_R_Deep = Y_C_Deep = 0
#----Fuzzy--------
Dy = 24
Dx = Turn_value = Speed_value = 0
#----RedDoor----
red_flag = False
CRMin = CRMax = crawl_cnt = 0
R_min = R_max = 0
B_left = B_right = B_min = B_max = XMax_one = XMin_one = XMin_two = XMax_two = 0
First_Reddoor = redoor_dis = False
imu_flag = slope_flag = True
BR_flag = BL_flag = False



def Image_Init():
    global Filter_Matrix, status,Xc, Dy1,Dy,WR, WL, Xb, Dx,Dx1, Xc_count, Xc_num, Deep_sum,Y_Deep_sum,Y_Deep_sum1,Y_Deep_sum2, L_Deep, R_Deep, R_min, R_max, B_min, B_max, B_left, B_right, XMax_one, XMin_one, XMin_two, XMax_two, y_move,C_Deep,R_line,L_line
    Filter_Matrix = []
    Dx1 = Dx = Xb = Xc = Xc_count = Xc_num = 0
    Dy1 = Dy = 24
    WR = WL = 0
    Deep_sum = L_Deep = R_Deep = C_Deep = 0
    Y_Deep_sum = Y_Deep_sum1 = Y_Deep_sum2 = 0 
    R_min = R_max = 0
    XMax_one = XMin_one = XMin_two = XMax_two = 00
    B_left = B_right = B_min = B_max = 0
    y_move = 0
    
#-----------------------------Parameter------------------------------------
def Fuzzy():
    global Dx,Turn,rule1,rule2,rule3,rule4,rule5,rule6,rule7,rule8,rule9,rule10,turn_ctrl,turning,Dx_value,Turn_value,CLEAR_SCREEN,Speed_value
    Dx = ctrl.Antecedent(np.arange(-16, 17, 1), 'Dx')
    Turn = ctrl.Consequent(np.arange(-13, 14, 1), 'Turn')
    Dx['R_Full'] = fuzz.trimf(Dx.universe, [-16, -16, -8])
    Dx['R_O'] = fuzz.trimf(Dx.universe, [-14, -12, -2])
    Dx['No'] = fuzz.trimf(Dx.universe, [-3, 0, 3])
    Dx['L_O'] = fuzz.trimf(Dx.universe, [2, 12, 14])
    Dx['L_Full'] = fuzz.trimf(Dx.universe, [8, 16, 16])
    Turn['T_Right'] = fuzz.trimf(Turn.universe, [-13, -13, -7])
    Turn['T_R'] = fuzz.trimf(Turn.universe, [-8, -8, -2])
    Turn['NoTurn'] = fuzz.trimf(Turn.universe, [-3, 0, 3])
    Turn['T_L'] = fuzz.trimf(Turn.universe, [2, 8, 8])
    Turn['T_Left'] = fuzz.trimf(Turn.universe, [7, 13, 13])
    rule1 = ctrl.Rule(Dx['L_Full'], Turn['T_Right'])
    rule2 = ctrl.Rule(Dx['L_O'], Turn['T_R'])
    rule3 = ctrl.Rule(Dx['No'], Turn['NoTurn'])
    rule4 = ctrl.Rule(Dx['R_O'], Turn['T_L'])
    rule5 = ctrl.Rule(Dx['R_Full'], Turn['T_Left'])
    # 建立模糊控制系統
    turn_ctrl = ctrl.ControlSystem([rule1, rule2, rule3,rule4,rule5])
    turning = ctrl.ControlSystemSimulation(turn_ctrl)
    # 執行模糊控制
    Dx_value = Dx1  
    turning.input['Dx'] = Dx_value
    turning.compute()
    # 解模糊得到控制輸出
    Turn_value = int(turning.output['Turn'])
    Dy = ctrl.Antecedent(np.arange(0, 25, 1), 'Dy')
    Speed = ctrl.Consequent(np.arange(0, 31, 1), 'Speed')
    Dy['C_OBS'] = fuzz.trimf(Dy.universe, [0, 0, 7])
    Dy['NC_OBS'] = fuzz.trimf(Dy.universe, [2, 12, 12])
    Dy['NOR_OBS'] = fuzz.trimf(Dy.universe, [6, 15, 15])
    Dy['NF_OBS'] = fuzz.trimf(Dy.universe, [10, 20, 20])
    Dy['F_OBS'] = fuzz.trimf(Dy.universe, [18, 24, 24])
    Speed['FAST'] = fuzz.trimf(Speed.universe, [20, 30, 30])
    Speed['N_F'] = fuzz.trimf(Speed.universe, [13, 25, 25])
    Speed['NOR_S'] = fuzz.trimf(Speed.universe, [5, 8, 13])
    Speed['N_S'] = fuzz.trimf(Speed.universe, [1, 5, 6])
    Speed['SLOW'] = fuzz.trimf(Speed.universe, [0, 0, 2])
    rule6 = ctrl.Rule(Dy['C_OBS'], Speed['SLOW'])
    rule7 = ctrl.Rule(Dy['NC_OBS'], Speed['N_S'])
    rule8 = ctrl.Rule(Dy['NOR_OBS'], Speed['NOR_S'])
    rule9 = ctrl.Rule(Dy['NF_OBS'], Speed['N_F'])
    rule10 = ctrl.Rule(Dy['F_OBS'], Speed['FAST'])
    # 建立模糊控制系統
    speed_ctrl = ctrl.ControlSystem([rule6, rule7, rule8,rule9,rule10])
    speeding = ctrl.ControlSystemSimulation(speed_ctrl)
    # 執行模糊控制
    Dy_value =  Dy1
    speeding.input['Dy'] = Dy_value
    speeding.compute()
    # 解模糊得到控制輸出
    Speed_value = 100*(int(speeding.output['Speed']))
    update_values()

def Normal_Obs_Parameter():
    global Filter_Matrix, L_line,R_line,Xc, WR, WL, Xb, Dx1, Xc_count, Xc_num,Filter_Matrix, Xc, Dy1,Y_Dy,B_Dy,Deep_sum,Y_Deep_sum,Y_Deep_sum1,Y_Deep_sum2,R_Deep, L_Deep, C_Deep,Y_R_Deep, Y_L_Deep, Y_C_Deep,B_R_Deep,B_L_Deep,B_C_Deep, red_flag, R_min, R_max, B_min, B_max, B_left, B_right, XMax_one, XMin_one, XMin_two, XMax_two,B_Deep_sum
    Focus_Matrix = [7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7] 
    Dx1 = 0
    Dy1 = 24
#-----------reddoor------------------
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
#----------------Y_line_DeepMatrix----------------
    for j in range (0, 32, 1):                  #黃色深度
        if deep.ya[j] < Y_Dy:
            Y_Dy = deep.ya[j]
        Y_Deep_sum += deep.ya[j]
        Y_L_Deep = deep.ya[2]
        Y_R_Deep = deep.ya[30]
        Y_C_Deep = deep.ya[16]
    for k in range (0, 15, 1):                  #黃色深度L
        Y_Deep_sum1 += deep.ya[k]
    for l in range (16, 32, 1):                 #黃色深度R
        Y_Deep_sum2 += deep.ya[l]

    for i in range (0, 32, 1):                  #濾波矩陣
        Filter_Matrix.append(0)
        Filter_Matrix[i] = (Focus_Matrix[i]) - deep.aa[i]
        if Filter_Matrix[i] >= 0 :
            Xc_count += 1
            Xc_num += i
            Xc = int(Xc_num) // int(Xc_count)
        else :
            Filter_Matrix[i] = 0
        WR += (32-i) * Filter_Matrix[i]
        WL += (i+1) * Filter_Matrix[i]
        if deep.aa[i] < Dy1:
            Dy1 = deep.aa[i]
        Deep_sum += deep.aa[i]
        L_Deep = deep.aa[4]
        R_Deep = deep.aa[28]
        C_Deep = deep.aa[16]
    if WL > WR:
        Xb = 31
    elif WL <= WR:
        Xb = 0
    if send.color_mask_subject_cnts[1] == 2 and send.color_mask_subject_YMax[1][0] < 200 and send.color_mask_subject_XMax[1][1] > 200:
        Dx1 = 0
        if Y_Deep_sum1 < Y_Deep_sum2 :
            L_line = True
            R_line = False
        elif Y_Deep_sum2 > Y_Deep_sum1:
            R_line = True
            L_line = False
    else:
        Dx1 = Xc - Xb
    update_values()
#------Init && Normal && Fuzzy------
def Init_Normal_Fuzzy():
    Image_Init()
    Normal_Obs_Parameter()
    Fuzzy()
#------Init && Normal------
def Init_Normal():
    Image_Init()
    Normal_Obs_Parameter()
#------Walking_status------
def Move(Straight_status = 0 ,x = -300 ,y = -200 ,z = 0 ,theta = -1  ,sensor = 0 ):
    global status
    if Straight_status == 0:            #stay
        send.sendContinuousValue(x,y,z,theta,sensor)
        status = "0_Stay"
    elif Straight_status == 11:
        status = "11_Turn"         #speed + turn  14 -9
        if Y_C_Deep < 12: 
            send.sendContinuousValue(x + (Speed_value*2),y,z,theta + Turn_value,sensor)
        else:
            send.sendContinuousValue(x + Speed_value,y,z,theta + Turn_value,sensor)
    elif Straight_status == 12:        #speed + imu
        status = "12_Imu_Fix"
        if red_flag == True:
            
            send.sendContinuousValue(x,y,z,theta + imu_angle,sensor)
        else:
            if Yaw_wen > 0:             #-13修右-11
                send.sendContinuousValue(x-100,y +700,z,theta + imu_angle,sensor)
            elif Yaw_wen <= 0:          #9修左
                send.sendContinuousValue(x-100,y -1000 ,z,theta + imu_angle,sensor)
    elif Straight_status == 122:        #YLine straight
        status = "122_Imu_Fix_Max_Speed"
        send.sendContinuousValue(2700,y-200 ,z,theta + imu_angle,sensor)
    elif Straight_status == 13:         #speed ++
        status = "13_Go_Straight"
        send.sendContinuousValue(x + Speed_value,y,z,-2,sensor)
#============================================================================#       
    elif Straight_status == 14:  #max speed
        status = "14_Max_Speed"
        send.sendContinuousValue(3000 ,-100 ,z ,-1 ,sensor)

    elif Straight_status == 15:  # small forward
        status = "15_Small_Forward"
        Slope_fix()
        send.sendContinuousValue(1000 ,-100 ,z ,-1 + slope_angle,sensor)

    elif Straight_status == 16:  #small back
        status = "16_Small_Back"
        Slope_fix()
        send.sendContinuousValue(-1200 ,0 ,z ,-1 + slope_angle ,sensor)
#---------------------Turn Head Parameter-------------------------#
    elif Straight_status == 21:  #turn right
        status = "21_Turn_Right"
        send.sendContinuousValue(-300 ,700 ,z ,-11 ,sensor)

    elif Straight_status == 22:  #right turn back
        status = "22_Turn_Right_Back"
        send.sendContinuousValue(-500 ,-1300 ,z ,9 ,sensor)

    elif Straight_status == 23:  #turn left
        status = "23_Turn_Left"
        send.sendContinuousValue(-300 ,-1100 ,z ,7 ,sensor)

    elif Straight_status == 24:  #left turn back
        status = "24_Turn_Left_Back"
        send.sendContinuousValue(-300 ,600 ,z ,-10 ,sensor)
        # send.sendContinuousValue(-500 ,900 ,z ,-11 ,sensor)

#--------------------turn head go straight------------------------#
    elif Straight_status == 25:  #turn right fix left
        status = "25_Turn_Right_Fix_Left"
        send.sendContinuousValue(1800 ,y + y_move ,z ,2 ,sensor)

    elif Straight_status == 26:  #turn right fix right
        status = "26_Turn_Right_Fix_Right"
        send.sendContinuousValue(1800 ,y + y_move ,z ,-8 ,sensor)

    elif Straight_status == 27:  #turn left fix right
        status = "27_Turn_Left_Fix_Right"
        send.sendContinuousValue(1800 ,y + y_move ,z ,-3 ,sensor)

    elif Straight_status == 28:  #turn left fix left
        status = "28_Turn_Left_Fix_Left"
        send.sendContinuousValue(1800 ,y + y_move ,z ,6 ,sensor) 

#------------------reddoor slope parameter------------------------#
    elif Straight_status == 31:  #Slope fix
        status = "31_Slope_Fix"
        send.sendContinuousValue(x,y,z,theta + slope_angle,sensor)

    elif Straight_status == 32:  #slope fix right
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
        send.sendContinuousValue(-300 + rx , -1000 ,0 ,-3 + slope_angle ,0) 
    
    elif Straight_status == 33:  #slope fix left
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
        send.sendContinuousValue(-400 + rx , 800 ,0 ,1 + slope_angle ,0) 

#--------------------Preturn Head Parameter-----------------------#
    elif Straight_status == 41:  #preturn left
        status = "41_Preturn_Left"
        send.sendContinuousValue(300,y,z,11,sensor)

    elif Straight_status == 42:  #preturn right
        status = "42_Preturn_Right"
        send.sendContinuousValue(300,y,z,-7,sensor)
    
    update_values()
    # time.sleep(0.5)
    # return status
def update_values():#更新數值
    global Deep_sum,Turn_value,Speed_value,red_flag,slope_angle,Straight_status,status,Dx1,Dy1,L_line,R_line,Yaw_wen,Y_Deep_sum1,Y_Deep_sum2,B_min,B_max,B_left, B_right,crawl_cnt,R_deep_sum,L_deep_sum
    # 移動光標到終端機的第一行
    # print("\033[H", end="")
    sys.stdout.write("\033[H")
    sys.stdout.write("\033[J")
    print("\033[1;31;40mDx and Dy\033[0m\t\033[1;31;40mTurn&&Speed\033[0m\t\033[1;31;40mIMU and Status\033[0m\n{} : {:<5}\t{}  : {:<5}\t{} : {:<5}\n{} : {:<5}\t{} : {:<5}\t{} :{:<5}\n=====================================".format("Dx",Dx1,"Turn",Turn_value,"Yaw_wen",Yaw_wen,"Dy",Dy1,"Speed",Speed_value,"Walking_Status",status))
    
    print("\033[1;31;40mB_OBS\033[0m\t\t\033[1;31;40mY_OBS\033[0m\t\t\033[1;31;40mR_OBS\033[0m")
    print("{} : {:<5}  {} : {:<5}\t{} :{:<5}\n=====================================".format("B_YMax",send.color_mask_subject_YMax[2][0],"Y_YMax",send.color_mask_subject_YMax[1][0],"R_YMax",send.color_mask_subject_YMax[5][0]))
    
    print("\033[1;31;40mRed_Door_Parameter\033[0m\t<< Red_Flag : {} >>".format(red_flag))
    print("{}  : {:<5}\t{}  : {:<5}\n{} : {:<5}\t{} : {:<5}\n{} : {:<5}\t{} : {:<5}\t{} : {:<5}\n{} : {:<5}\t{} : {:<5}\n=====================================".format("B_min",B_min,"B_max",B_max,"XMin_1",XMin_one,"XMax_1",XMin_two,"XMin_2",XMax_one,"XMax_2",XMax_two,"Slope_angle",slope_angle,"B_Left",B_left,"B_Right",B_right))
    #send.color_mask_subject_XMin[2][0]
    print("\033[1;31;40m Crawl_Parameter \033[0m")
    if crawl_cnt < 8:
        print("crawl_cnt : {}\n=====================================".format(crawl_cnt))
    else:
        print("crawl_cnt : Finish\n=====================================")
    print("\033[1;31;40mY_Line Flag\033[0m\nR_line : {}\t\tR_Y_Deep : {}\nL_Line : {}\t\tL_Y_Deep : {}\n=====================================".format(R_line,Y_Deep_sum2,L_line,Y_Deep_sum1))
    print("Deep_sum: {}".format(Deep_sum))
    print("Rdeep: {}".format(R_deep_sum))
    print("Ldeep: {}".format(L_deep_sum))

    
    
def Turn_Head():
    global R_deep_sum, L_deep_sum, L_Deep, R_Deep, y_move,TurnHead_Flag,B_C_Deep
    Move(Straight_status = 0)
    if R_line == False and L_line == False : 
        time.sleep(1)
        send.sendHeadMotor(1,1447,100)
        send.sendHeadMotor(2,1600,100)
        send.sendHeadMotor(1,1447,100)
        send.sendHeadMotor(2,1600,100)
        send.sendHeadMotor(1,1447,100)
        send.sendHeadMotor(2,1600,100)
        time.sleep(1.3) 
        Init_Normal()
        R_deep_sum = Deep_sum
        send.sendHeadMotor(1,2647,100)
        send.sendHeadMotor(2,1600,100)
        send.sendHeadMotor(1,2647,100)
        send.sendHeadMotor(2,1600,100)
        send.sendHeadMotor(1,2647,100)
        send.sendHeadMotor(2,1600,100)
        time.sleep(1.8)
        Init_Normal()
        L_deep_sum = Deep_sum
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,1550,100)
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,1550,100)
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,1550,100)
        time.sleep(1)
    else :
        R_deep_sum = 0
        L_deep_sum = 0

    if (R_deep_sum > L_deep_sum) or (L_line == True):        #右轉
        
        Init_Normal_Fuzzy()
        while ( B_C_Deep > 3 ):#靠近障礙物
            Init_Normal_Fuzzy()
            Move(Straight_status = 15) 
        while ( B_C_Deep < 5 ):#遠離障礙物
            Init_Normal_Fuzzy()
            Move(Straight_status = 16) 
        get_IMU()                
        while abs(Yaw_wen) < 70:#靠近後右旋轉至90度
            Init_Normal()
            get_IMU()
            Move(Straight_status = 21)
        send.sendHeadMotor(1,2647,100)
        send.sendHeadMotor(2,1550,100) 
        time.sleep(1)
        Init_Normal_Fuzzy()
        send.sendContinuousValue(0,0,0,0,0)  
        while abs(Dx1) >= 1:#直走且imu修正 
            Init_Normal_Fuzzy()
            get_IMU()
            if (R_Deep != 24) and (R_Deep <= 14) :                  #轉頭後直走 平移修正
                y_move = -800
            elif (R_Deep != 24) and (R_Deep > 14) :
                y_move = 600
            if abs(Yaw_wen) > 87 :          #視步態更動
                Move(Straight_status = 25)
            else :
                Move(Straight_status = 26)
            if C_Deep == 24 and R_Deep == 24:#MRT
                Move(Straight_status = 15) 
                time.sleep(1)
                break
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,1550,100) 
        time.sleep(1)
        Init_Normal_Fuzzy()
        get_IMU()         
        while abs(Yaw_wen) > 60:#右轉回正
            Init_Normal()
            get_IMU()
            Move(Straight_status = 22)
    elif (L_deep_sum > R_deep_sum) or (R_line == True):         #左轉
        Init_Normal_Fuzzy()                 
        while ( B_C_Deep  > 3 ):#靠近障礙物
            Init_Normal_Fuzzy()
            Move(Straight_status = 15)                
        while ( B_C_Deep < 5 ):#遠離障礙物
            Init_Normal_Fuzzy()
            Move(Straight_status = 16) 
        get_IMU()              
        while abs(Yaw_wen) < 65:#靠近後轉至90度
            Init_Normal_Fuzzy()
            get_IMU()
            Move(Straight_status = 23)
        send.sendHeadMotor(1,1447,100)
        send.sendHeadMotor(2,1550,100) 
        time.sleep(1)
        Init_Normal_Fuzzy()
        send.sendContinuousValue(0,0,0,0,0)                          
        while  abs(Dx1) >= 1 :#直走且imu修正
            Init_Normal_Fuzzy()
            get_IMU()
            if (L_Deep != 24) and (L_Deep <= 14) :                  #轉頭後直走 平移修正
                y_move = 400
            elif (L_Deep != 24) and (L_Deep > 14) :
                y_move = -400
            if abs(Yaw_wen) > 85 :          #視步態更動
                Move(Straight_status = 27)
            else :
                Move(Straight_status = 28) 
            if C_Deep == 24 and L_Deep ==24:#MRT
                Move(Straight_status = 15) 
                time.sleep(1)
                break
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,1550,100) 
        time.sleep(1)
        Init_Normal_Fuzzy()
        get_IMU()             
        while abs(Yaw_wen) > 50:#左轉回正    
            Init_Normal_Fuzzy()
            get_IMU()
            Move(Straight_status = 24)
    TurnHead_Flag = False
    update_values()
    # time.sleep(0.5)

def Crawl():
    global Straight_status,crawl_cnt,C_Deep,L_Deep,R_Deep,CRMax,CRMin,B_min,B_max,B_left,B_right
    while send.color_mask_subject_YMax[5][0] < CRMin or send.color_mask_subject_YMax[5][0] > CRMax or CRMin <= send.color_mask_subject_YMax[5][0] <= CRMax:
        if(send.color_mask_subject_YMax[5][0] < CRMin):            #前進修正
            Slope_fix()
            Move(Straight_status = 15)
            print('crawlllllll forwardddddddddd')
        elif(send.color_mask_subject_YMax[5][0] > CRMax):          #後退修正
            Slope_fix()
            Move(Straight_status = 16)
            print('crawlllllll backkkkkkkk')
        elif CRMin <= send.color_mask_subject_YMax[5][0] <= CRMax:    #爬                              
            while abs(deep.slope) > 0.03:#不平行紅門時修斜率
                Slope_fix()
                Move(Straight_status = 31)
            print('CCCCCCCCCCCCCCCCRWAL')
            send.sendContinuousValue(0, 0 , 0 , 0 , 0) 
            time.sleep(2000)
            # send.sendBodyAuto(0,0,0,0,1,0)
            # time.sleep(2)
            # send.sendBodySector(299)
            # time.sleep(3.5)
            # send.sendBodySector(123)
            # time.sleep(9)
            # time.sleep(0.5)
            while crawl_cnt < 3:                #避免門下建模有問題 固定爬三次才抬頭
                # send.sendBodySector(456)
                # time.sleep(2.8)
                # time.sleep(0.3)
                crawl_cnt += 1
            send.color_mask_subject_YMax[1][0] = 0
            send.color_mask_subject_YMax[2][0] = 0
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,2500,100)
            # time.sleep(1)
            while crawl_cnt < 8:                   #邊爬邊判斷是否離障礙物太近
                Init_Normal()
                # time.sleep(0.1)
                if (send.color_mask_subject_YMax[2][0] >= 85 and send.color_mask_subject_size[2][0] > 5000) or (send.color_mask_subject_YMax[1][0] >= 60 and send.color_mask_subject_size[1][0] > 5000):
                    break
                else:
                    # send.sendBodySector(456)
                    # time.sleep(2.8)
                    #time.sleep(0.1)
                    crawl_cnt += 1
            # send.sendBodySector(1113)
            # time.sleep(14.4)
            # time.sleep(1.5)
            # send.sendBodySector(299)
            # time.sleep(1.5)
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,1550,100)
            # time.sleep(0.5)
            #send.sendBodySector(15)#小白
            # send.sendBodySector(16)#小黑
            # time.sleep(2.5)
            send.sendBodyAuto(0,0,0,0,1,0)
            # time.sleep(1)
            break
    update_values()
    # time.sleep(0.5)

def Slope_fix():
    global slope_angle
    if deep.slope > 0:          #fix to l
        if  deep.slope >= 2:
            slope_angle = 0
        elif deep.slope >= 1:
            slope_angle = 12
        elif 1 > deep.slope >= 0.3:
            slope_angle = 11
        elif 0.3 > deep.slope >= 0.2:
            slope_angle = 10
        elif 0.2 > deep.slope >= 0.15:
            slope_angle = 9
        elif 0.15 > deep.slope >= 0.1:
            slope_angle = 5
        elif 0.1 > deep.slope >= 0.05:
            slope_angle = 4
        elif 0.05 > deep.slope >= 0.03:
            slope_angle = 3
        elif 0.03 > deep.slope >= 0:
            slope_angle = 0
    elif deep.slope <= 0:       #fix to r
        if  deep.slope <= -2:
            slope_angle = 0
        elif -1 >= deep.slope:
            slope_angle = -14
        elif -0.3 >= deep.slope > -1:
            slope_angle = -13
        elif -0.2 >= deep.slope > -0.3:
            slope_angle = -12
        elif -0.15 >= deep.slope > -0.2:
            slope_angle = -8
        elif -0.1 >= deep.slope > -0.15:
            slope_angle = -7
        elif -0.05 >= deep.slope > -0.1:
            slope_angle = -6
        elif -0.03 >= deep.slope > -0.05:
            slope_angle = -5
        elif 0 >= deep.slope > -0.03:
            slope_angle = 0
    if send.color_mask_subject_size[5][0] ==0 :
        slope_angle = 0
    update_values()
    # time.sleep(0.5)
    # return slope_angle
def IMU_Yaw_ini():
    global  Yaw_wen
    Yaw_wen = 0
    update_values()
    # time.sleep(0.5)

def get_IMU():
    global Yaw_wen
    Yaw_wen = send.imu_value_Yaw
    update_values()
    # time.sleep(0.5)
    # return Yaw_wen

def IMU_Angle():
    global imu_angle
    if Yaw_wen > 0:         #fix to r
        if Yaw_wen >= 90:
            imu_angle = -11
        elif 90 > Yaw_wen >= 60:
            imu_angle = -10
        elif 60 > Yaw_wen >= 45:
            imu_angle = -9
        elif 45 > Yaw_wen >= 20:
            imu_angle = -9
        elif 20 > Yaw_wen >= 10:
            imu_angle = -7
        elif 10 > Yaw_wen >= 5:
            imu_angle = -6
        elif 5 > Yaw_wen >= 2:
            imu_angle = -5
        elif 2 > Yaw_wen >= 0:
            imu_angle = 0
    elif Yaw_wen <= 0:      #fix to l
        if -90 >= Yaw_wen:
            imu_angle = 10
        elif -60 >= Yaw_wen > -90:
            imu_angle = 9
        elif -45 >= Yaw_wen > -60:
            imu_angle = 8
        elif -20 >= Yaw_wen > -45:
            imu_angle = 7
        elif -10 >= Yaw_wen > -20:
            imu_angle = 6
        elif -5 >= Yaw_wen > -10:
            imu_angle = 5
        elif -2 >= Yaw_wen > -5:
            imu_angle = 4
        elif 0 >= Yaw_wen > -2:
            imu_angle = 0
    update_values()
    # time.sleep(0.5)
    # return imu_angle

if __name__ == '__main__':
    try:
        deep = deep_calculate()
        send = Sendmessage() 
        while not rospy.is_shutdown():
            if send.is_start == True:
                #==============================image===============================
                # Focus_Matrix = [6, 7, 8, 8, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 8, 8, 7, 6]
                Focus_Matrix = [7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7]#6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 9, 9, 9, 9, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6
                # YYDS = 1   #Y_Line_Deep 1  4
                CRMax = 25 #R_Max       50 25
                CRMin = 15 #R_MIn       40 15
                Init_Normal_Fuzzy()
                #=============================strategy=============================
                if walking == False:                        #指撥後初始動作

                    PreTurn_L = False
                    # PreTurn_L = True
                    PreTurn_R = False
                    # PreTurn_R = True

                    time.sleep(0.5)
                    send.sendHeadMotor(1,2048,100)
                    send.sendHeadMotor(2,1500,100)
                    time.sleep(0.2)
                    #send.sendBodySector(15)     #收手 小白
                    # send.sendBodySector(16)     #收手 小黑
                    # send.sendBodySector(1218)   #長腳
                    # time.sleep(2.5)
                    send.sendBodyAuto(0,0,0,0,1,0)
                    time.sleep(1.5) 
                walking = True
                if PreTurn_L == True :                      #指定初始向左旋轉
                    get_IMU()
                    while abs(Yaw_wen) < 40:
                        get_IMU()
                        Move(Straight_status = 41)
                    PreTurn_L = False
                elif PreTurn_R == True :                        #指定初始向右旋轉 70
                    get_IMU()
                    while abs(Yaw_wen) < 40:
                        get_IMU()
                        Move(Straight_status = 42)
                    PreTurn_R = False
                if red_flag == True:                    #若有紅門
                    print('In Reddoor')
                    if First_Reddoor == False :
                        First_Reddoor = True
                        send.sendHeadMotor(1,2048,100)
                        send.sendHeadMotor(2,1800,100)
                        # time.sleep(0.5)
                        send.sendContinuousValue(0,0,0,0,0)
                    elif First_Reddoor == True :
                        while send.color_mask_subject_YMax[5][0] >= 230:
                            Init_Normal_Fuzzy()
                            Move(Straight_status = 16)                      
                        while abs(deep.slope) > 0.03 and (slope_flag == True):#不平行紅門時修斜率
                            Slope_fix()
                            Move(Straight_status = 31)
                        slope_flag = False
                        send.sendHeadMotor(1,2048,100)
                        send.sendHeadMotor(2,1700,100)
                        # time.sleep(0.5)
                        if (send.color_mask_subject_YMax[5][0] < 150) and (redoor_dis == False) :     #前後距離修正（值越大離門越近） 55/65   離紅門太遠時前進
                            Slope_fix()
                            Move(Straight_status = 15)
                            pass
                        elif (send.color_mask_subject_YMax[5][0] > 160) and (redoor_dis == False) :   #前後距離修正（值越大離門越近） 55/65   離紅門太近時候退
                            Slope_fix()
                            Move(Straight_status = 16)
                            pass
                        else :                          #判斷須左移或右移 都不需則進爬
                            redoor_dis = True
                            if R_min < 2 and R_max > 315 :
                                if (B_max == 0 and B_min == 0 and B_left <= 35 and BR_flag == True) or (B_max == 0 and B_min == 0 and B_right > 275 and BL_flag == True) or (B_max == 0 and B_min == 0 and B_right == 0 and B_left == 0):
                                    while abs(deep.slope) > 0.03:# and (slope_flag == True):
                                        Slope_fix()
                                        Move(Straight_status = 31)
                                    Crawl()
                                elif (B_min < 2 and B_max > 20):
                                    print('move R 11111')
                                    BR_flag = True
                                    BL_flag = False
                                    Move(Straight_status = 32)
                                elif (B_max > 315 and B_min < 300):
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
                    if Dy1 < 24:
                        Init_Normal_Fuzzy()
                        YL_Deep_sum = Y_Deep_sum1
                        YR_Deep_sum = Y_Deep_sum2
                        if R_line == True :
                            if YL_Deep_sum > YR_Deep_sum :
                                R_line = True
                                # pass
                            elif (YL_Deep_sum < YR_Deep_sum) or (YR_Deep_sum > 350) :
                                R_line = False
                                # pass
                        elif L_line == True :
                            if YL_Deep_sum < YR_Deep_sum :
                                L_line = True
                            elif (YL_Deep_sum > YR_Deep_sum) or (YL_Deep_sum > 350) :
                                L_line = False
                                
                        if 14 > Dx1 > 2 :        #turn right
                            # if (send.color_mask_subject_YMax[2][0] >= 170) and ( abs(Yaw_wen) <= 5 ) and imu_back == False and abs(Dx1) > 3 and C_Deep != 24:        #離障礙物太近-->後退
                            #     while (send.color_mask_subject_YMax[2][0] >= 170):
                            #         Init_Normal_Fuzzy()
                            #         if send.color_mask_subject_YMax[2][0] >0 :
                            #             break
                                # imu_back = True
                            if ( abs(Yaw_wen) > 5 and IMU_ok == False ) and Dx1 >= 5 :       #IMU修正
                                get_IMU()
                                IMU_Angle()
                                Move(Straight_status = 12)
                                Init_Normal_Fuzzy()
                            else:
                                Init_Normal_Fuzzy()
                                Move(Straight_status = 11)

                            if abs(Yaw_wen) <= 5 :
                                IMU_ok = True
                        elif -2 > Dx1 > -14 :     #turn left
                            # if (send.color_mask_subject_YMax[2][0] >= 170) and ( abs(Yaw_wen) <= 5 ) and imu_back == False and abs(Dx1) > 3 and C_Deep != 24:        #離障礙物太近-->後退
                            #     while (send.color_mask_subject_YMax[2][0] >= 170):
                            #         Init_Normal_Fuzzy()
                            #         Move(Straight_status = 16) 
                            #         if send.color_mask_subject_YMax[2][0] > 0 :
                            #             break
                            #     imu_back = True
                            if ( abs(Yaw_wen) > 5 and IMU_ok == False ) and Dx1 <= -7 :      #IMU修正
                                get_IMU()
                                IMU_Angle()
                                Move(Straight_status = 12)
                                Init_Normal_Fuzzy()
                            else:
                                Init_Normal_Fuzzy()
                                Move(Straight_status = 11)

                            if abs(Yaw_wen) <= 5 :
                                IMU_ok = True
                        elif (Dx1 < 17 and Dx1 >= 14) or (Dx1 <= -14 and Dx1 > -17) :
                            IMU_Angle()
                            if (send.color_mask_subject_YMax[2][0] >= 170) and ( abs(Yaw_wen) <= 5 ) and imu_back == False and abs(Dx1) > 3 and C_Deep != 24:        #離障礙物太近-->後退
                                while (send.color_mask_subject_YMax[2][0] >= 170):
                                    Init_Normal_Fuzzy()
                                    Move(Straight_status = 16) 
                                    if send.color_mask_subject_YMax[2][0] >0 :
                                        break
                                imu_back = True                   
                            while ( abs(Yaw_wen) > 3 or IMU_ok == False) :#IMU修正
                                IMU_Angle()
                                get_IMU()
                                Move(Straight_status = 12)
                                Init_Normal_Fuzzy()
                                if abs(Yaw_wen) < 3:        #轉頭策略
                                    if ( B_L_Deep < 15 ) and ( B_R_Deep < 15 ) and ( B_C_Deep < 20 ):
                                        TurnHead_Flag = True
                                        Turn_Head()
                                        IMU_ok = True
                                        break
                                    else :
                                        break
                                else:
                                    print('IMU NOT OK')
                                    pass
                            if (abs(Yaw_wen) < 3 and IMU_ok == True):
                                IMU_Angle()
                                get_IMU()
                                Move(Straight_status = 12)
                                Init_Normal_Fuzzy()
                                if abs(Yaw_wen) < 5: 
                                    if ( B_L_Deep < 15 ) and ( B_R_Deep < 15 ) and ( B_C_Deep < 20 ):
                                        Turn_Head()
                                    else :
                                        if Dx1 > 0 :
                                            Move(Straight_status = 11)
                                        elif Dx1 < 0 :
                                            Move(Straight_status = 11)
                                else:
                                    print('IMU NOT OK')
                                    pass
                        elif (1 >= Dx1 >= -1) or abs(Dx1) >= 17:                  #最高速直走
                            IMU_Angle()
                            get_IMU()
                            Init_Normal_Fuzzy()
                            Move(Straight_status = 14)
                            if Dx1 == 0 :
                                IMU_ok = False
                                imu_back = False
                    elif Dy1 == 24:
                        Init_Normal_Fuzzy()
                        Move(Straight_status = 13)
            if send.is_start == False:
                if walking == True:
                    send.sendContinuousValue(0,0,0,0,0)
                    time.sleep(1.5) 
                    send.sendBodyAuto(0,0,0,0,1,0)
                    # time.sleep(1)
                    # send.sendBodySector(299)
                    walking = False
                    update_values() 
                    IMU_Yaw_ini()
    except rospy.ROSInterruptException:
        pass
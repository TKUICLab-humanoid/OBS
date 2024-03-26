#!/usr/bin/env python
#coding=utf-8
# from turtle import st
import rospy
import numpy as np #import NumPy陣列
from hello1 import Sendmessage
# import sys
# sys.path.append('/home/iclab/Desktop/kid_hurocup/src/strategy')
# from Python_API import Sendmessage
from ddd import deep_calculate
# from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tku_msgs.msg import camera
import cv2 
import sys
import time
import math

deep            = deep_calculate()  #在ddd
send            = Sendmessage()     #在hello1
CRMAX           = 63 # red door 前後修正3 值越大離門越近 #68
CRMIN           = 63 # red door 前後修正3 值越大離門越近 #68
HEAD_HEIGHT     = 1550 #頭高，位置為馬達目標刻度，2048為正朝前方
FOCUS_MATRIX    = [7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 10, 10, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7]
#=========================================== 
MAX_FORWARD_X         = 3000                                                     
MAX_FORWARD_Y         = 200                                                            
MAX_FORWARD_THETA     = 0                                     
#===========================================                 
TURN_RIGHT_X            = 500                                                     
TURN_RIGHT_Y            = 700                                                     
TURN_RIGHT_THETA        =   -4  
#=========================================== 
IMU_RIGHT_X            =  300 
IMU_RIGHT_Y            =  800           
#===========================================                                         
TURN_LEFT_X             =  200                                                    
TURN_LEFT_Y             =  -1000                                                     
TURN_LEFT_THETA         =    5  
#=========================================== 
IMU_LEFT_X            =   200 
IMU_LEFT_Y            =   -1000   
#===========================================                                             

class Walk(): #步態、轉彎、直走速度、IMU
    def __init__(self):
        self.image = Normal_Obs_Parameter()
        
    def move(self, action_id, z=0, sensor= 0):
        self.image.calculate()
        imu_flag = self.get_imu() < 0     #判斷是否<0 
        slope_x_fix             = 500 if self.image.red_y_max < 35 else -500 if self.image.red_y_max > 45 else 0            #red door 平移 前後修正 值越大越遠
        right_straight_y        = -200 if self.image.center_deep <= 3  else 500 if self.image.center_deep >= 7 else 0     #turn head 右轉 直走 值越大越遠             
        left_straight_y         = -200 if self.image.center_deep <= 4  else 500 if self.image.center_deep >= 9 else 0     #turn head 左轉 直走 值越大越遠
        straight_90degree_fix   = -2 if ((imu_flag and abs(self.get_imu()) < 90) or (not imu_flag and abs(self.get_imu()) > 90)) else 2   #turn head 保持90度直走         
        turn_x                  =   self.straight_speed()*2 if self.image.yellow_center_deep < 12 else self.straight_speed()  
        turn_direction_x        =   TURN_RIGHT_X if self.get_imu() > 0 else TURN_LEFT_X  # fix_angle for turn_x
        actions             = { 'stay'                  : {'x':  500,                 'y':  200,               'theta': 0 },
                                'max_speed'             : {'x':  MAX_FORWARD_X,     'y':   MAX_FORWARD_Y,   'theta': MAX_FORWARD_THETA },
                                'small_back'            : {'x': -1500,              'y':  300,             'theta': -1 },
                                'small_forward'         : {'x':  1500,              'y':  300,                'theta': 0 },
                                'imu_fix'               : {'x': IMU_RIGHT_X if self.get_imu() > 0 else IMU_LEFT_X,  'y': IMU_RIGHT_Y if self.get_imu() > 0 else IMU_LEFT_Y, 'theta': self.imu_angle()  },
                                # 'slope_fix'             : {'x': IMU_RIGHT_X-200 if self.get_imu() > 0 else IMU_LEFT_X-200,  'y': IMU_RIGHT_Y if self.get_imu() > 0 else IMU_LEFT_Y, 'theta': self.slope()      },
                                'slope_fix'             : {'x': 100 if deep.slope > 0 else 500,                   'y':    -500 if deep.slope > 0 else 700,           'theta': self.slope()},
                                'imu_right_translate'   : {'x': 0 + slope_x_fix, 'y': -1000,            'theta': -2 + self.imu_angle()      },
                                'imu_left_translate'    : {'x':  100+ slope_x_fix, 'y':  1500,      'theta': 2 + self.imu_angle()      },
                                'slope_right_translate' : {'x': 0 + slope_x_fix, 'y': -1000,            'theta': -2 + self.slope()      },
                                'slope_left_translate'  : {'x':  100+ slope_x_fix, 'y':  1500,      'theta': 2 + self.slope()      },
                                'dx_turn'               : {'x': TURN_RIGHT_X if self.image.deep_x > 0 else TURN_LEFT_X,       'y':  TURN_RIGHT_Y if self.image.deep_x > 0 else TURN_LEFT_Y,     'theta': self.turn_angle()  },
                                'turn_right_for_wall'   : {'x': TURN_RIGHT_X,       'y':  TURN_RIGHT_Y,     'theta': TURN_RIGHT_THETA  },
                                'turn_right_back'       : {'x': IMU_RIGHT_X if self.get_imu() > 0 else IMU_LEFT_X,  'y': IMU_RIGHT_Y if self.get_imu() > 0 else IMU_LEFT_Y,            'theta': TURN_LEFT_THETA                 },#.
                                'turn_left_for_wall'    : {'x': TURN_LEFT_X,        'y':  TURN_LEFT_Y,      'theta': TURN_LEFT_THETA   },
                                'turn_left_back'        : {'x': IMU_RIGHT_X if self.get_imu() > 0 else IMU_LEFT_X,  'y': IMU_RIGHT_Y if self.get_imu() > 0 else IMU_LEFT_Y,            'theta': TURN_RIGHT_THETA                },#.
                                'face_right_forward'    : {'x': MAX_FORWARD_X,    'y':  MAX_FORWARD_Y + right_straight_y ,    'theta': MAX_FORWARD_THETA + straight_90degree_fix    },
                                # 'right_right'         : {'x': SMALL_FORWARD_X,    'y':  SMALL_FORWARD_Y + straight_y_fix,     'theta': SMALL_FORWARD_THETA + straight_90degree_fix    },
                                'face_left_forward'     : {'x': MAX_FORWARD_X,    'y':  MAX_FORWARD_Y + left_straight_y,     'theta': MAX_FORWARD_THETA + straight_90degree_fix   },
                                # 'left_left'           : {'x': SMALL_FORWARD_X,    'y':  SMALL_FORWARD_Y + straight_y_fix,     'theta': SMALL_FORWARD_THETA + straight_90degree_fix    },
                                'preturn_left'          : {'x': TURN_LEFT_X,        'y':  TURN_LEFT_Y,      'theta': TURN_LEFT_THETA   },
                                'preturn_right'         : {'x': TURN_RIGHT_X,       'y':  TURN_RIGHT_Y,     'theta': TURN_RIGHT_THETA  }}
        action              = actions.get(action_id,None)   
        if action is not None:
            x              = action['x']
            y              = action['y']
            theta          = action['theta']
            send.sendContinuousValue(x, y, z, theta, sensor)
        print(action_id)
        # print(self.slope())

    def imu_yaw_ini(self):
        self.imu_yaw = 0

    def get_imu(self):
        self.imu_yaw = send.imu_value_Yaw
        return self.imu_yaw 

    def turn_angle(self):   #一般 旋轉角度
        self.image.calculate()
        turn_ranges = [ (17, -4), 
                        (12, -3), 
                        (8,  -3), 
                        (6,  -2), 
                        (4,  -2), 
                        (2,  -1),  
                        (0,   0),
                        (-2,  2),
                        (-4,  3),
                        (-6,  3),
                        (-8,  3),
                        (-12, 4),
                        (-17, 5)]
        for turn_range in turn_ranges:           
            if  self.image.deep_x >= turn_range[0]:
                return turn_range[1]
        return 0                                 
    
    def imu_angle(self):      #一般 imu修正角度
        imu_ranges = [  (180,  -4),
                        (90,  -4), 
                        (60,  -4), 
                        (45,  -4), 
                        (20,  -4), 
                        (10,  -3), 
                        (5,   -3), 
                        (2,   -2), 
                        (0,    0),
                        (-2,    2),
                        (-5,    3),
                        (-10,   3),
                        (-20,   4),
                        (-45,   4),
                        (-60,   4),
                        (-90,   5),
                        (-180,   5)]
        for imu_range in imu_ranges:           
            if self.imu_yaw >= imu_range[0]:
                return imu_range[1]
        return 0

    def slope(self):    #red 斜率修正角度
#-------------------fix to l---------------------
        if deep.slope > 0:          
            slopel_ranges = [(1,     3), 
                             (0.5,   2), 
                             (0.3,   2), 
                             (0.2,   1), 
                             (0.15,  1), 
                             (0.1,   1), 
                             (0.06,  1), 
                             (0.03,  1), 
                             (0,     0)]
            for slopel_range in slopel_ranges:
                if deep.slope >= slopel_range[0]:
                    return slopel_range[1]
            return 0
#--------------------fix to r--------------------
        elif deep.slope <= 0:     
            slopel_ranges = [(-1,     -3), 
                             (-0.5,   -2), 
                             (-0.3,   -2), 
                             (-0.2,   -1), 
                             (-0.15,  -1), 
                             (-0.1,   -1), 
                             (-0.06,  -1), 
                             (-0.03,  -1), 
                             (0,       0)]
            for slopel_range in slopel_ranges:
                if deep.slope >= slopel_range[0]:
                    return slopel_range[1]
            return 0 
        if send.color_mask_subject_size[5][0] < 5000 :
            slope_angle = 0
        print(slope_angle)
        return slope_angle

    def straight_speed(self):   #一般避障 前進速度
        self.image.calculate()
        speed_ranges = [(24,    3000), 
                        (20,    2700), 
                        (16,    2200), 
                        (14,    1800), 
                        (12,    1400), 
                        (8,     1000), 
                        (6,      600), 
                        (3,      200), 
                        (0,        0)]
        for speed_range in speed_ranges:
            if self.image.deep_y >= speed_range[0]: #最小深度>=24
                return speed_range[1]
        return 0
    

class Normal_Obs_Parameter: #計算各種深度
    def __init__(self):
        self.line_at_left               = False
        self.line_at_right              = False
        self.line_at_right_test         = False 
        self.line_at_left_test          = False
        self.at_reddoor_flag            = False
        self.deep_y                     = 24
        self.deep_x                     = 0
        self.yellow_center_deep         = 0       #黃色中心深度值
        self.y_move                     = 0
        self.deep_sum                   = 0
        self.y_deep_sum                 = 0
        self.y_deep_left_sum            = 0
        self.y_deep_right_sum           = 0
        self.left_deep                  = 0
        self.right_deep                 = 0
        self.center_deep                = 0
        self.red_x_min                  = 0
        self.red_x_max                  = 0
        self.blue_leftside              = 0
        self.blue_rightside             = 0
        self.red_y_max                  = 0
        self.b_y_max                    = 0
        self.b_x_min                    = 0
        self.b_x_max                    = 0

    def calculate(self):
        self.red_y_max = send.color_mask_subject_YMax[5][0]
        if send.color_mask_subject_size[5][0] > 5000: #如果紅門夠大 #有紅時計算紅門資訊
            self.at_reddoor_flag = True #紅門旗標打開
            self.red_x_min = send.color_mask_subject_XMin[5][0] #紅門最左邊
            self.red_x_max = send.color_mask_subject_XMax[5][0] #紅門最右邊

            self.blue_rightside     = 0 #藍門資訊做歸0
            self.blue_leftside      = 0
            self.b_x_min            = 0
            self.b_x_max            = 0

            if send.color_mask_subject_cnts[2] == 1: #畫面裡只有一個藍色障礙物(只看到紅門下的一面牆)
                self.b_x_min = send.color_mask_subject_XMin[2][0] #藍牆最左邊
                self.b_x_max = send.color_mask_subject_XMax[2][0] #藍牆最右邊
            elif send.color_mask_subject_cnts[2] == 2: #畫面裡有兩個藍色障礙物(看到紅門下的兩面牆)
                xmax_one                = send.color_mask_subject_XMax[2][0]
                xmin_one                = send.color_mask_subject_XMin[2][0]
                xmin_two                = send.color_mask_subject_XMin[2][1]
                xmax_two                = send.color_mask_subject_XMax[2][1]
                self.blue_rightside     = max(xmin_one, xmin_two) #可以算出紅門中間的洞洞
                self.blue_leftside      = min(xmax_one, xmax_two) #可以算出紅門中間的洞洞

        else : 
            self.at_reddoor_flag = False #if 紅門不夠大，紅門旗標關起來(一般避障)
    #----------------Blue_DeepMatrix-----------------
            self.b_y_max        = send.color_mask_subject_YMax[2][0] #藍色YMax
            self.b_deep_y       = min(deep.ba) #藍色深度最小值(離最近)
            self.b_deep_sum     = sum(deep.ba) #藍色所有深度和
            self.b_left_deep    = deep.ba[2]   #第2行深度(藍)
            self.b_right_deep   = deep.ba[30]  #第30行深度(倒數第二行)(藍)
            self.b_center_deep  = deep.ba[16]  #第16行深度(中間)(藍)
    #----------------Y_line_DeepMatrix---------------
            self.y_deep_y           = min(deep.ya) #黃色深度最小值(離最近)
            self.y_deep_sum         = sum(deep.ya)
            self.y_left_deep        = deep.ya[2]
            self.y_right_deep       = deep.ya[30]
            self.yellow_center_deep = deep.ya[16]
            self.y_deep_left_sum    = sum(deep.ya[0:15]) #黃色左邊深度總和(0-15行)(黃)
            self.y_deep_right_sum   = sum(deep.ya[16:31]) #黃色右邊深度總和(16-31行)(黃)
    #----------------Filter_matrix-------------------
            filter_matrix          = [max(0, a - b) for a, b in zip(FOCUS_MATRIX, deep.aa)] #在Foucus area內的障礙物的最大值(V矩陣最大值->最近)
            x_center_num           = sum(i for i, num in enumerate(FOCUS_MATRIX - np.array(deep.aa)) if num >= 0)
            x_center_cnt           = np.sum(np.array(FOCUS_MATRIX) - np.array(deep.aa) >= 0) 
            x_center               = (x_center_num / x_center_cnt) if x_center_cnt > 0 else 0
            left_weight_matrix     = list(range(32))            #0~31 #建立一個包含0到31的整數的列表
            right_weight_matrix    = list(range(31,-1,-1))      #31~0 #建立一個包含31到0的整數的列表
            right_weight           = np.dot(filter_matrix,  right_weight_matrix)#內積
            left_weight            = np.dot(filter_matrix,  left_weight_matrix)
            self.deep_y                 = min(deep.aa)
            self.deep_sum               = sum(deep.aa)
            self.deep_sum_l             = sum(deep.aa[0:16]) #左邊深度總和(全)
            self.deep_sum_r             = sum(deep.aa[17:32])#右邊深度總和(全)
            self.left_deep              = deep.aa[4] #第4行深度(全)(設2的話跟藍色重疊???)
            self.right_deep             = deep.aa[28] #第28行深度(倒數第四行)(全色)
            self.center_deep            = deep.aa[16] #第16行深度(中間)(全色)
            x_boundary             = 31 if left_weight > right_weight else 0 #boundary point

            if send.color_mask_subject_cnts[1] == 2 and send.color_mask_subject_YMax[1][0] > 140 and send.color_mask_subject_YMax[1][1] > 140:
                self.deep_x = 0
            # if send.color_mask_subject_cnts[1] == 2 and send.color_mask_subject_YMax[1][0] > 60 and send.color_mask_subject_YMax[1][1] > 60: #yellow yellow 值越大 越不容易直走
                #如果黃黃通道接藍障會不會看到3個黃色?
                # if abs(send.color_mask_subject_XMax[1][0] - send.color_mask_subject_XMin[1][1] ) > 70: #黃色通道
                # self.deep_x = 0 #dx歸0
                # elif send.color_mask_subject_XMax[1][0] > 160 and send.color_mask_subject_XMin[1][1] > 160 and abs(send.color_mask_subject_XMax[1][0] - send.color_mask_subject_XMin[1][1] ) < 70: #黃線在右邊
                #     self.deep_x = x_center - x_boundary
                #     print('rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr')
                #     #self.line_at_right_test = True 
                #     #self.line_at_left_test  = False
                #     self.line_at_right = True 
                #     self.line_at_left  = False
                # elif send.color_mask_subject_XMax[1][0] < 160 and send.color_mask_subject_XMin[1][1] < 160 and abs(send.color_mask_subject_XMax[1][0] - send.color_mask_subject_XMin[1][1] ) < 70: #黃線在左邊
                #     self.deep_x = x_center - x_boundary
                #     print('llllllllllllllllllllllllllllllllll')
                #     #self.line_at_right_test = True 
                #     #self.line_at_left_test  = False
                #     self.line_at_right = False
                #     self.line_at_left  = True                               
            # else:    
            # print("wl = ",left_weight)
            # print("wr = ",right_weight)
            # print("xc = ",x_center)
            # print("xb = ",x_boundary)
            else:
                self.deep_x = x_center - x_boundary #dx=Xc-Xb
            print("b_cnt = ",send.color_mask_subject_cnts[2])
            print("b_l_max = ",send.color_mask_subject_XMax[2][0])
            print("b_r_min = ",send.color_mask_subject_XMin[2][1])
class Obs: #各種避障動作
    def __init__(self):
        self.image          = Normal_Obs_Parameter()
        self.walk           = Walk()
        self.blue_at_left           = False
        self.blue_at_right          = False
        self.redoor_distence        = False
        self.need_fix_slope         = True
        self.first_reddoor          = False
        self.start_walking          = False
        self.imu_ok                 = False
        self.need_imu_back          = True
        self.line_at_right_single   = False
        self.line_at_left_single    = False
        self.door_at_left           = False
        self.door_at_right          = False
        self.left_deep_sum          = 0
        self.right_deep_sum         = 0
        self.crawl_cnt              = 0

    def red_door(self): #前後修正1 -> 修斜率 -> 前後修正2 -> 平移 -> 前後修正3 -> 趴下
        
        if not self.first_reddoor:
            self.first_reddoor = True
            send.sendHeadMotor(1,2048,100) #頭在中間
            send.sendHeadMotor(2,HEAD_HEIGHT + 150,100)
            time.sleep(0.2)
            send.sendContinuousValue(0, 0 , 0 , 0 , 0)
        # while True:
        #     self.image.calculate() #計算障礙物的各種參數(深度、左右權重、dx...)
        #     print("red_y_max:", (self.image.red_y_max))
        self.image.calculate()
        while self.image.red_y_max > 150 : #red door 前後修正1 值越大離門越近 #離紅門太近了
            self.image.calculate()
            self.walk.move('small_back')

        if abs(deep.slope) > 0.03 :     # red door 修斜率
            while abs(deep.slope) > 0.03 :
                # self.walk.move('slope_fix')
                self.walk.move('slope_fix') #self.walk.move('imu_fix') #根據斜率修正IMU
            # self.need_fix_slope = False
        self.image.calculate()
        if (self.image.red_y_max < 95) :     #red door 前後修正2 值越大離門越近 
            while self.image.red_y_max < 95 : #離紅門太遠了
                self.image.calculate()
                self.walk.move('small_forward') 
        elif (self.image.red_y_max > 95) or self.image.b_center_deep == 0:   #red door  前後修正2 值越大離門越近 
            while self.image.red_y_max > 95 or self.image.b_center_deep == 0:
                self.image.calculate()
                self.walk.move('small_back')
        # self.redoor_distence = True
        while 1 :     
            self.image.calculate()
            #print("red_size", send.color_mask_subject_size[5][0])
            #print("red_num", send.color_mask_subject_cnts[5])
            if (self.image.red_x_min < 2 and self.image.red_x_max > 315) and send.color_mask_subject_size[5][0] > 5000: #紅門在眼前
                self.image.calculate()
                print("xxxxxxxxxxxxxxxxxxxxxxxxx")
                print('b_cnt = ',send.color_mask_subject_cnts[2])
                if (self.image.b_x_max == 0 and self.image.b_x_min == 0 and self.image.blue_leftside <= 60 and self.image.blue_rightside > 250 and self.blue_at_right ) or (self.image.b_x_max == 0 and self.image.b_x_min == 0 and self.image.blue_leftside <= 60 and self.image.blue_rightside > 250 and self.blue_at_left ) : #or (self.image.b_x_max == 0 and self.image.b_x_min == 0 and self.image.blue_rightside == 0 and self.image.blue_leftside == 0)
                    print
                    while abs(deep.slope) > 0.03 :
                        self.walk.move('slope_fix')
                        # print('333333333333333333333333')
                    self.crawl()
                    break
                elif (self.image.b_x_min < 2 and self.image.b_x_max > 50):
                    self.image.calculate()
                    self.blue_at_right = True
                    self.blue_at_left = False
                    self.walk.move('slope_right_translate')
                    print("333333333333333333333")
                elif (self.image.b_x_max > 315 and self.image.b_x_min < 265):
                    self.image.calculate()
                    self.blue_at_left = True
                    self.blue_at_right = False
                    self.walk.move('slope_left_translate')
                    print("4444444444444444444444")
                # elif (self.image.b_x_max == 0 and self.image.b_x_min == 0):
                elif (send.color_mask_subject_cnts[2] == 2):
                    # if(send.color_mask_subject_XMax[2][0]<60 and send.color_mask_subject_XMin[2][1]<295):
                    self.image.calculate()
                    if self.image.blue_rightside < 268 and self.image.blue_leftside <54:
                        self.image.calculate()
                        self.blue_at_left = True
                        self.blue_at_right = False
                        self.walk.move('slope_left_translate')
                        print("555555555555555")
                    # elif(send.color_mask_subject_XMax[2][0]>20 and send.color_mask_subject_XMin[2][1]>265):
                    elif self.image.blue_rightside > 268 and self.image.blue_leftside > 54:
                        self.image.calculate()
                        self.blue_at_left = True
                        self.blue_at_right = False
                        self.walk.move('slope_right_translate')
                        print("666666666666666")
                else :
                    self.image.calculate()
                    print("77777777777777777777")
                    if self.blue_at_right :
                        self.walk.move('slope_right_translate')
                    elif self.blue_at_left :
                        self.walk.move('slope_left_translate')
                self.image.calculate()
            # elif (self.image.red_x_min < 2 and self.image.red_x_max < 315) and send.color_mask_subject_size[5][0] > 5000: #紅門在面前偏左
            elif (self.image.red_x_min < 2 and self.image.red_x_max < 315):
                self.image.calculate()
                self.walk.move('slope_left_translate')

            # elif (self.image.red_x_min > 2 and self.image.red_x_max > 315) and send.color_mask_subject_size[5][0] > 5000: #紅門在面前偏右
            elif (self.image.red_x_min > 2 and self.image.red_x_max > 315):
                self.image.calculate()
                self.walk.move('slope_right_translate') 
            else :
                self.image.calculate()
                self.walk.move('stay')
                time.sleep(1)
                send.sendHeadMotor(1,1517,180) #頭往右轉
                send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                send.sendHeadMotor(1,1517,180)
                send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                send.sendHeadMotor(1,1517,180)
                send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                time.sleep(1.5) 
                self.image.calculate()
                if send.color_mask_subject_size[5][0] > 5000:
                    self.door_at_right = True
                    self.door_at_left = False
                send.sendHeadMotor(1,2599,180) #頭往左轉
                send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                send.sendHeadMotor(1,2599,180)
                send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                send.sendHeadMotor(1,2599,180)
                send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                time.sleep(1.5)
                self.image.calculate()
                if send.color_mask_subject_size[5][0] > 5000:
                    self.door_at_left = True
                    self.door_at_right = False
                send.sendHeadMotor(1,2048,180) #頭轉正
                send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                send.sendHeadMotor(1,2048,180)
                send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                send.sendHeadMotor(1,2048,180)
                send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
                time.sleep(0.3)
                self.image.calculate()
                if self.door_at_right :
                    while self.image.red_x_min < 160:
                        self.image.calculate()
                        self.walk.move('imu_right_translate')
                elif self.door_at_left:
                    while self.image.red_x_max > 160:
                        self.image.calculate()
                        self.walk.move('imu_left_translate')

    def crawl(self):
        self.image.calculate()
        while 1 :
            self.image.calculate()
            if (self.image.b_x_max == 0 and self.image.b_x_min == 0 and self.image.blue_leftside <= 20 and self.blue_at_right ) or (self.image.b_x_max == 0 and self.image.b_x_min == 0 and self.image.blue_rightside > 260 and self.blue_at_left ) or (self.image.b_x_max == 0 and self.image.b_x_min == 0 and self.image.blue_rightside == 0 and self.image.blue_leftside == 0):
                while abs(deep.slope) > 0.03 :
                    self.walk.move('slope_fix')
                    print('44444444444444444444444444444')
                break
            elif (self.image.b_x_min < 2 and self.image.b_x_max > 50):
                print("8888888888888888888")
                self.blue_at_right = True
                self.blue_at_left = False
                self.walk.move('slope_right_translate')
            elif (self.image.b_x_max > 315 and self.image.b_x_min < 265):
                self.blue_at_left = True
                self.blue_at_right = False
                self.walk.move('slope_left_translate')
            else :
                if self.blue_at_right :
                    self.walk.move('slope_right_translate')
                elif self.blue_at_left :
                    self.walk.move('slope_left_translate')
        self.image.calculate()
        if((self.image.red_y_max) < CRMIN):            #離紅門太遠
            while(self.image.red_y_max < CRMIN):          
                self.image.calculate()
                self.walk.slope()
                self.walk.move('small_forward')
                print("CRMIN:", self.image.red_y_max)
        elif(self.image.red_y_max > CRMAX):          #離紅門太近
            while(self.image.red_y_max > CRMAX):         
                self.image.calculate()
                print("CRMAX:", self.image.red_y_max)
                self.walk.slope()
                self.walk.move('small_back')  
        while abs(deep.slope) > 0.03: #紅門太斜
            self.walk.slope()
            self.walk.move('slope_fix')
        send.sendContinuousValue(0, 0 , 0 , 0 , 0) 
        time.sleep(1)
        send.sendBodyAuto(0,0,0,0,1,0) #mode = 1為continue步態 #停下來
        time.sleep(2)  
        send.sendBodySector(333) #執行motion儲存的sector
        time.sleep(6)
        # send.sendBodySector(222) #執行motion儲存的sector
        # time.sleep(2.2)
        # send.sendBodySector(29) #stand
        # time.sleep(0.5)
        send.sendBodySector(1111)
        time.sleep(8)
        while self.crawl_cnt < 3:    #count 3次            
            send.sendBodySector(2222)
            time.sleep(3)
            # time.sleep(0.3)
            self.crawl_cnt += 1
        send.color_mask_subject_YMax[1][0] = 0 #黃色YMax =0
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,2400,100) #頭往上抬
        time.sleep(1)
        while self.crawl_cnt < 7:   #cnt3數到7(4次)
            send.sendBodySector(2222)
            time.sleep(3.5)
            self.crawl_cnt += 1               
            self.image.calculate()
            # print("blue_ymax   = ",self.b_y_max) #change
            if (send.color_mask_subject_YMax[2][0] >= 35 and send.color_mask_subject_size[2][0] > 5000) or (send.color_mask_subject_YMax[1][0] >= 35 and send.color_mask_subject_size[1][0] > 5000): #爬到黃色或藍色夠近或夠大
                break
        if self.crawl_cnt > 6 :
            send.sendBodySector(3333)
            time.sleep(11)
            send.sendBodySector(29)    
            time.sleep(0.5)
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT,100)
            time.sleep(1)
            send.sendBodySector(111)
            time.sleep(3.5)
            send.sendBodySector(1218)
            time.sleep(0.5)
            send.sendBodyAuto(0,0,0,0,1,0) 
        else :
            send.sendBodySector(3333)
            time.sleep(14)
            send.sendBodySector(29)    
            time.sleep(0.5)
            send.sendBodySector(1218)
            time.sleep(0.5)
            send.sendBodySector(18)
            time.sleep(0.5)
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT,100)
            time.sleep(2)
            send.sendBodyAuto(0,0,0,0,1,0)

    def turn_head(self):
        self.walk.move('stay')
        if not self.image.line_at_right and not self.image.line_at_left: 
            time.sleep(1)
            send.sendHeadMotor(1,1517,180) #頭往右轉
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            send.sendHeadMotor(1,1517,180)
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            send.sendHeadMotor(1,1517,180)
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            time.sleep(1.5) 
            if send.color_mask_subject_YMax[1][0] > 220 and send.color_mask_subject_size[1][0] > 5000: #黃色夠近夠大
                self.line_at_right_single = True
            if send.color_mask_subject_YMax[5][0] > 220 and send.color_mask_subject_size[5][0] > 5000: #紅色夠近夠大
                self.door_at_right = True
                self.door_at_left = False
            self.right_deep_sum = sum(deep.aa) #filter_sum_aa #右邊深度總和
            print(self.line_at_right_single)
            send.sendHeadMotor(1,2599,180) #頭往左轉
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            send.sendHeadMotor(1,2599,180)
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            send.sendHeadMotor(1,2599,180)
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            time.sleep(1.5)
            if send.color_mask_subject_YMax[1][0] > 220 and send.color_mask_subject_size[1][0] > 3000:
                self.line_at_left_single = True
            if send.color_mask_subject_YMax[5][0] > 220 and send.color_mask_subject_size[5][0] > 5000:
                self.door_at_left = True
                self.door_at_right = False
            self.left_deep_sum = sum(deep.aa) #左邊深度總和
            print(self.line_at_left_single)
            send.sendHeadMotor(1,2048,180) #頭回中間
            send.sendHeadMotor(2,HEAD_HEIGHT,180)
            send.sendHeadMotor(1,2048,180)
            send.sendHeadMotor(2,HEAD_HEIGHT,180)
            send.sendHeadMotor(1,2048,180)
            send.sendHeadMotor(2,HEAD_HEIGHT,180)
            time.sleep(0.3)
        else : #已知黃線在左/右側
            self.right_deep_sum = 0
            self.left_deep_sum = 0
        if (self.right_deep_sum > self.left_deep_sum) or self.image.line_at_left or self.line_at_left_single or self.door_at_right: #turn head 右轉
            self.image.calculate()
            if (self.image.b_center_deep > 7):                   #turn head 右轉 前後修正 越大越遠
                while ( self.image.b_center_deep > 7):
                    # print("aaaaaaaaaaaa")
                    # print('c_deep:', self.image.b_center_deep)
                    self.image.calculate()
                    self.walk.move('small_forward')
            elif ( self.image.b_center_deep < 6):                 #turn head 右轉 前後修正 越大越遠
                while ( self.image.b_center_deep < 6 ):
                    self.image.calculate()
                    self.walk.move('small_back')
            if abs(self.walk.get_imu()) < 75:                     #turn head 旋轉角度  右轉 越大轉越多
                while abs(self.walk.get_imu()) < 75:
                    self.walk.move('turn_right_for_wall')
            send.sendHeadMotor(1,2647,100) #身體面相右，頭往左轉看牆
            send.sendHeadMotor(2,1550,100) 
            time.sleep(0.5)
            self.image.calculate()
            send.sendContinuousValue(0, 0 , 0 , 0 , 0)
            while abs(self.image.deep_x) >= 4 and send.color_mask_subject_size[5][0] < 20000:     #turn head 右轉 直走 結束位置 越小站的越外面
                self.image.calculate()
                self.walk.move('face_right_forward')
            send.sendHeadMotor(1,2048,100) #頭轉正
            send.sendHeadMotor(2,HEAD_HEIGHT,100) 
            time.sleep(0.5)
            self.image.calculate()
            if abs(self.walk.get_imu()) > 50:             #turn head 右轉 回正 數字越小越正對
                while abs(walk.get_imu()) > 50: 
                    self.walk.move('turn_right_back')
            if self.door_at_right :
                self.red_door()
        elif (self.left_deep_sum > self.right_deep_sum) or self.image.line_at_right or self.line_at_right_single or self.door_at_left:         #turn head 左轉
            self.image.calculate() 
            if (self.image.b_center_deep > 8):                   #turn head 左轉 前後修正 越大越遠
                while ( self.image.b_center_deep > 8 ):
                    print( self.image.b_center_deep)
                    print("bbbbbbbbbbbb")
                    self.image.calculate()
                    self.walk.move('small_forward') 
            elif ( self.image.b_center_deep < 7 ):                #turn head 左轉 前後修正 越大越遠 
                while ( self.image.b_center_deep < 7 ):
                    self.image.calculate()
                    self.walk.move('small_back') 
            if abs(self.walk.get_imu()) < 70:                           #turn head 旋轉角度  左轉 越大轉越多
                while abs(self.walk.get_imu()) < 70:
                    self.walk.move('turn_left_for_wall')
                    print('llllllllllllllllllllllllllll')
            send.sendHeadMotor(1,1447,100) #身體面相左，頭往右轉看牆
            send.sendHeadMotor(2,1550,100) 
            time.sleep(0.5)
            self.image.calculate()
            send.sendContinuousValue(0, 0 , 0 , 0 , 0)                         
            while  abs(self.image.deep_x) >= 4 and send.color_mask_subject_size[5][0] < 20000:        #20000 #turn head 左轉  直走 結束位置 越小站的越外面
                self.image.calculate()
                self.walk.move('face_left_forward')
            send.sendHeadMotor(1,2048,100)#頭轉正
            send.sendHeadMotor(2,HEAD_HEIGHT,100) 
            time.sleep(0.5)
            self.image.calculate()
            if abs(self.walk.get_imu()) > 50:               #turn head 左轉 回正 數字越小越正對
                while abs(self.walk.get_imu()) > 50:    
                    self.walk.move('turn_left_back')
            if self.door_at_left :
                self.red_door()

    def main(self):
        if send.is_start : #策略主指撥開關
            # while True:
            self.image.calculate() #計算障礙物的各種參數(深度、左右權重、dx...)
                        # self.image.calculate()
            print('dx:', self.image.deep_x)
            # print('y_YMAX1:', send.color_mask_subject_YMax[1][0])
            # print('y_YMAX2:', send.color_mask_subject_YMax[1][1])
            # print('y_XMAX1:', send.color_mask_subject_XMax[1][0])
            # print('y_XMin2:', send.color_mask_subject_XMin[1][1])
            # print('y_cnt:', send.color_mask_subject_cnts[1])
            # print('imu:', self.walk.get_imu())
            print('red_y_max:', self.image.red_y_max)
            
            # print('line_at_right:', self.image.line_at_right)
            # print('line_at_left:', self.image.line_at_left)
            # print('blue_rightside:', self.image.blue_rightside)
            # print('blue_leftside:', self.image.blue_leftside)
                
            
            
            # print('a=',self.image.b_x_min)
            # print('b=',self.image.b_x_max)
        #=============================strategy=============================
            if not self.start_walking :                        #指撥後初始動作
                # self.walk.imu_yaw_ini() #imu歸0 (imu_yaw = 0)
                #================================================
                self.preturn_left = False
                # self.preturn_left = True
                #================================================
                self.preturn_right = False
                # self.preturn_right = True
                #================================================
                send.sendHeadMotor(1,2048,100) #頭部初始動作
                send.sendHeadMotor(2,HEAD_HEIGHT  ,100)
                send.sendHeadMotor(1,2048,100)
                send.sendHeadMotor(2,HEAD_HEIGHT  ,100)
                send.sendHeadMotor(1,2048,100)
                send.sendHeadMotor(2,HEAD_HEIGHT  ,100)
                time.sleep(0.5)
                send.sendBodyAuto(0,0,0,0,1,0) #步態呼叫
                self.start_walking = True
            if self.preturn_left:
                while abs(self.walk.get_imu()) < 5:
                    self.walk.move('preturn_left')
                    rospy.loginfo(f'imu =  {self.walk.get_imu()}')
                self.preturn_left = False
            elif self.preturn_right:
                while abs(self.walk.get_imu()) < 35:
                    self.walk.move('preturn_right')
                    rospy.loginfo(f'imu =  {self.walk.get_imu()}')
                self.preturn_right = False
            if self.image.at_reddoor_flag: #進紅門
                self.red_door()
                pass
            else : #進一般避障
                if self.image.deep_y < 24:
                    self.image.calculate() #計算障礙物的各種參數(深度、左右權重、dx...)


                    # self.yyyyy = 0
                    # for i in range(0,16):
                    #     if (deep.ya[i] !=24) and (abs(self.walk.get_imu()) < 45):
                    #         self.yyyyy += 1
                    # self.image.calculate()
                    # # print(self.yyyyy)
                    # if self.yyyyy >= 14:
                    #     while 1 :
                    #         self.walk.move('max_speed')
                    if self.image.line_at_right :
                        if self.image.y_deep_left_sum > self.image.y_deep_right_sum :
                            self.line_at_right = True
                            self.imu_ok = True
                        elif (self.image.y_deep_left_sum < self.image.y_deep_right_sum) or (self.image.y_deep_right_sum > 350) :
                            self.image.line_at_right = False
                            self.imu_ok = False
                    elif self.image.line_at_left :
                        if self.image.y_deep_left_sum < self.image.y_deep_right_sum :
                            self.image.line_at_left = True
                            self.imu_ok = True
                        elif (self.image.y_deep_left_sum > self.image.y_deep_right_sum) or (self.image.y_deep_right_sum > 350) :
                            self.image.line_at_left = False
                            self.imu_ok = False
                    
                    if 13 > self.image.deep_x > 4 : #deep_x = dx  #normal turn 右轉 範圍越大越容易旋轉 三個地方要調整 大的數字不動
                        self.walk.straight_speed()
                        # if (self.image.b_y_max >= 190) and ( abs(self.walk.get_imu()) <= 5 ) and (self.need_imu_back)  and (abs(self.image.deep_x) > 3) and (self.image.center_deep != 24):        #離障礙物太近-->後退
                        #     while (self.image.b_y_max >= 150):
                        #         self.image.calculate()
                        #         self.walk.move('small_back') 
                        #         if self.image.b_y_max > 0 :
                        #             break
                        #     self.need_imu_back = False
                        if ((abs(self.walk.get_imu()) > 5) and (not self.imu_ok)) and (self.image.deep_x >= 6) :       #normal turn 右轉 數值越大 越不容易 修imu
                            self.walk.move('imu_fix')
                        else:
                            self.walk.move('dx_turn')

                        if abs(self.walk.get_imu()) <= 5 :
                            self.imu_ok = True
                    elif -4 > self.image.deep_x > -13 :     #normal turn 左轉 範圍越大越容易旋轉 三個地方要調整 大的數字不動
                        self.walk.straight_speed()
                        # if (self.image.b_y_max >= 190) and ( abs(self.walk.get_imu()) <= 5 ) and (self.need_imu_back) and (abs(self.image.deep_x) > 3) and (self.image.center_deep != 24):        #離障礙物太近-->後退
                        #     while (self.image.b_y_max >= 150):
                        #         self.image.calculate()
                        #         self.walk.move('small_back') 
                        #         if self.image.b_y_max > 0 :
                        #             break
                        #     self.need_imu_back = False
                        if ((abs(self.walk.get_imu()) > 5) and (not self.imu_ok)) and (self.image.deep_x <= -6) :   #normal turn 左轉 數值越大 越不容易 修imu   
                            self.walk.move('imu_fix')
                        else:
                            self.walk.move('dx_turn')

                        if abs(self.walk.get_imu()) <= 2 :
                            self.imu_ok = True
                    elif (self.image.deep_x < 17 and self.image.deep_x >= 13) or (self.image.deep_x <= -13 and self.image.deep_x > -17) :
                        # while True:
                            # # self.image.calculate()
                            # print("dx = ",self.image.deep_x)
                        self.image.calculate()
                        # print("dx = ",self.image.deep_x)
                        # self.walk.move('stay')

                        if (self.image.b_y_max >= 170) and ( abs(self.walk.get_imu()) <= 5 ) and (self.need_imu_back) and (abs(self.image.deep_x) > 3) and (self.image.center_deep != 24) :        #離障礙物太近-->後退
                            while (self.image.b_y_max >= 160):
                                self.image.calculate()
                                self.walk.move('small_back') 
                                if self.image.b_y_max > 0 : #怕機器人的晃動會卡在small back的迴圈
                                    break
                            self.need_imu_back = False
                        if ( abs(self.walk.get_imu()) > 2) and (not self.imu_ok) :                   #IMU修正
                            while ( abs(self.walk.get_imu()) > 2) and (not self.imu_ok) :
                                self.walk.move('imu_fix')
                                self.image.calculate()
                                if abs(self.walk.get_imu()) < 2:        #轉頭策略
                                    if ( self.image.left_deep < 15 ) and ( self.image.right_deep < 15 ) and ( self.image.center_deep < 15 ): #左中右深度皆小於15
                                        self.turn_head()
                                        self.imu_ok = True
                                        break
                                    # elif (send.color_mask_subject_XMax[2][0] > 50) and (send.color_mask_subject_XMin[2][1] < 260) :
                                    #     self.deep_x = 0
                                    elif  self.image.deep_sum_l >= self.image.deep_sum_r :
                                        while abs(self.walk.get_imu()) < 25:    
                                            self.walk.move('turn_left_for_wall')
                                            print('dx:', self.image.deep_x)
                                            print('deep_sum_l:', self.image.deep_sum_l)
                                        self.imu_ok = True
                                    elif  self.image.deep_sum_l < self.image.deep_sum_r :
                                        while abs(self.walk.get_imu()) < 25:    
                                            self.walk.move('turn_right_for_wall')
                                            print('dx:', self.image.deep_x)
                                            print('deep_sum_r:', self.image.deep_sum_r)
                                        self.imu_ok = True
                                else:
                                    pass
                        elif (abs(self.walk.get_imu()) < 2) and (self.imu_ok):
                            if ( self.image.left_deep < 15 ) and ( self.image.right_deep < 15 ) and ( self.image.center_deep < 15 ):
                                self.turn_head()
                                self.imu_ok = True
                            # elif (send.color_mask_subject_XMax[2][0] > 40) and (send.color_mask_subject_XMin[2][1] < 280):
                            #     self.deep_x = 0
                            elif  self.image.deep_sum_l >= self.image.deep_sum_r :
                                while abs(self.walk.get_imu()) < 25:    
                                    self.walk.move('turn_left_for_wall')
                                    print('dx:', self.image.deep_x)
                                    print('deep_sum_l:', self.image.deep_sum_l)
                                self.imu_ok = True
                            elif  self.image.deep_sum_l < self.image.deep_sum_r :
                                while abs(self.walk.get_imu()) < 25:    
                                    self.walk.move('turn_right_for_wall')
                                    print('dx:', self.image.deep_x)
                                    print('deep_sum_r:', self.image.deep_sum_r)
                                self.imu_ok = True

                    elif (4 >= self.image.deep_x >= -4) or (abs(self.image.deep_x) >= 17):                  #normal turn 直走 跟一般旋轉值要相等
                        self.walk.move('max_speed') #dx一定小於等於16，但有可能測出17
                        if self.image.deep_x == 0 :
                            self.imu_ok = False
                            self.need_imu_back = True
                            
                elif self.image.deep_y == 24: #畫面完全沒有障礙物
                    self.walk.move('max_speed')
                    
        if not send.is_start :
            # send.sendSensorReset(1,1,1) #將(Roll, Pitch, Yaw) 歸零
            print(deep.slope)
            # print("blue_ymax   = ",send.color_mask_subject_YMax[2][0])
            # print("yellow_ymax = ",send.color_mask_subject_YMax[1][0])
            # print('ready')
            if self.start_walking :
                send.sendContinuousValue(0,0,0,0,0) #x,y,z,theta填入walking介面移動數值
                send.sendBodyAuto(0,0,0,0,1,0) #mode=1為continue步態
                time.sleep(0.5)
                self.start_walking = False
            # send.sendContinuousValue(0,0,0,0,0)
            # self.walk.move('stay')

if __name__ == '__main__':

    try:
        aaaa = rospy.init_node('talker', anonymous=True)
        walk = Walk()
        strategy = Obs()
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            # if send.Web :
            #     pass
            # if not send.Web :
            strategy.main()
            r.sleep()     
    except rospy.ROSInterruptException:
        pass

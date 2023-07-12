#!/usr/bin/env python
#coding=utf-8
# from turtle import st
import rospy
import numpy as np
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

deep            = deep_calculate()
send            = Sendmessage()
CRMAX           = 15 # red door 前後修正3 值越大離門越近
CRMIN           = 25 # red door 前後修正3 值越大離門越近
HEAD_HEIGHT     = 1550
FOCUS_MATRIX    = [7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 9, 9, 9, 10, 10, 11, 11, 10, 10, 9, 9, 9, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7]
#=========================================== 
MAX_FORWARD_X         = 3000                                                     
MAX_FORWARD_Y         = 0                                                            
MAX_FORWARD_THETA     = 0                                     
#===========================================                  
TURN_RIGHT_X            = -200                                                     
TURN_RIGHT_Y            =  1100                                                     
TURN_RIGHT_THETA        =   -6  
#=========================================== 
IMU_RIGHT_X            = -300 
IMU_RIGHT_Y            =  700           
#===========================================                                         
TURN_LEFT_X             =  0                                                    
TURN_LEFT_Y             =  -900                                                     
TURN_LEFT_THETA         =    6  
#=========================================== 
IMU_LEFT_X            =   100 
IMU_LEFT_Y            =   -600   
#===========================================                                             

class Walk():
    def __init__(self):
        self.image = Normal_Obs_Parameter()
        
    def move(self, action_id, z=0, sensor= 0):
        self.image.calculate()
        imu_flag = self.get_imu() < 0     #判斷是否<0
        slope_x_fix             = 500 if self.image.red_y_max < 50 else -500 if self.image.red_y_max > 55 else 0            #red door 平移 前後修正 值越大越遠
        right_straight_y        = -200 if self.image.center_deep <= 8  else 600 if self.image.center_deep >= 9 else 0     #turn head 右轉 直走 值越大越遠             
        left_straight_y         = -200 if self.image.center_deep <= 10  else 600 if self.image.center_deep >= 11 else 0     #turn head 左轉 直走 值越大越遠
        straight_90degree_fix   = -2 if ((imu_flag and abs(self.get_imu()) < 90) or (not imu_flag and abs(self.get_imu()) > 90)) else 2   #turn head 保持90度直走         
        turn_x                  =   self.straight_speed()*2 if self.image.yellow_center_deep < 12 else self.straight_speed()  
        turn_direction_x        =   TURN_RIGHT_X if self.get_imu() > 0 else TURN_LEFT_X  # fix_angle for turn_x
        actions             = { 'stay'                  : {'x':  -100,                 'y':  0,               'theta': 0 },
                                'max_speed'             : {'x':  MAX_FORWARD_X,     'y':   MAX_FORWARD_Y,   'theta': MAX_FORWARD_THETA },
                                'small_back'            : {'x': -1500,              'y':  -100,             'theta': 0 },
                                'small_forward'         : {'x':  1500,              'y':  0,                'theta': 0 },
                                'imu_fix'               : {'x': IMU_RIGHT_X if self.get_imu() > 0 else IMU_LEFT_X,  'y': IMU_RIGHT_Y if self.get_imu() > 0 else IMU_LEFT_Y, 'theta': self.imu_angle()  },
                                'slope_fix'             : {'x': IMU_RIGHT_X-200 if self.get_imu() > 0 else IMU_LEFT_X-200,  'y': IMU_RIGHT_Y if self.get_imu() > 0 else IMU_LEFT_Y, 'theta': self.slope()      },
                                'imu_right_translate' : {'x': 0 + slope_x_fix, 'y': -1000,            'theta': -2 + self.imu_angle()      },
                                'imu_left_translate'  : {'x':  100+ slope_x_fix, 'y':  1000,      'theta': 1 + self.imu_angle()      },
                                'slope_right_translate' : {'x': 0 + slope_x_fix, 'y': -1000,            'theta': -2 + self.slope()      },
                                'slope_left_translate'  : {'x':  100+ slope_x_fix, 'y':  1000,      'theta': 1 + self.slope()      },
                                'turn_right'            : {'x': TURN_RIGHT_X,       'y':  TURN_RIGHT_Y,     'theta': TURN_RIGHT_THETA  },
                                'turn_right_back'       : {'x': IMU_RIGHT_X if self.get_imu() > 0 else IMU_LEFT_X,  'y': IMU_RIGHT_Y if self.get_imu() > 0 else IMU_LEFT_Y,            'theta': TURN_LEFT_THETA                 },#.
                                'turn_left'             : {'x': TURN_LEFT_X,        'y':  TURN_LEFT_Y,      'theta': TURN_LEFT_THETA   },
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
        print(self.slope())

    def imu_yaw_ini(self):
        self.imu_yaw = 0

    def get_imu(self):
        self.imu_yaw = send.imu_value_Yaw
        return self.imu_yaw 

    def turn_angle(self):   #一般 旋轉角度
        self.image.calculate()
        turn_ranges = [ (17, -5), 
                        (12, -5), 
                        (8,  -4), 
                        (6,  -3), 
                        (4,  -3), 
                        (2,  -2),  
                        (0,   0),
                        (-2,  2),
                        (-4,  3),
                        (-6,  3),
                        (-8,  4),
                        (-12, 5),
                        (-17, 5)]
        for turn_range in turn_ranges:           
            if  self.image.deep_x >= turn_range[0]:
                return turn_range[1]
        return 0                                 
    
    def imu_angle(self):      #一般 imu修正角度
        imu_ranges = [  (180,  -5),
                        (90,  -5), 
                        (60,  -5), 
                        (45,  -4), 
                        (20,  -4), 
                        (10,  -4), 
                        (5,   -3), 
                        (2,   -2), 
                        (0,    0),
                        (-2,    2),
                        (-5,    3),
                        (-10,   4),
                        (-20,   4),
                        (-45,   4),
                        (-60,   5),
                        (-90,   5),
                        (-180,   5)]
        for imu_range in imu_ranges:           
            if self.imu_yaw >= imu_range[0]:
                return imu_range[1]
        return 0

    def slope(self):    #red 斜率修正角度
#-------------------fix to l---------------------
        if deep.slope > 0:          
            slopel_ranges = [(2,     3), 
                             (1,     3), 
                             (0.3,   2), 
                             (0.2,   2), 
                             (0.15,  2), 
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
            slopel_ranges = [(-2,     -3), 
                             (-1,     -3), 
                             (-0.3,   -2), 
                             (-0.2,   -2), 
                             (-0.15,  -2), 
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

    def straight_speed(self):   #一般 前進速度
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
            if self.image.deep_y >= speed_range[0]:
                return speed_range[1]
        return 0
    

class Normal_Obs_Parameter:
    def __init__(self):
        self.line_at_left               = False
        self.line_at_right              = False
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
        if send.color_mask_subject_size[5][0] > 5000:                      #有紅時計算紅門資訊
            self.at_reddoor_flag = True
            self.red_x_min = send.color_mask_subject_XMin[5][0] 
            self.red_x_max = send.color_mask_subject_XMax[5][0]

            self.blue_rightside     = 0
            self.blue_leftside      = 0
            self.b_x_min            = 0
            self.b_x_max            = 0

            if send.color_mask_subject_cnts[2] == 1:
                self.b_x_min = send.color_mask_subject_XMin[2][0]
                self.b_x_max = send.color_mask_subject_XMax[2][0]
            elif send.color_mask_subject_cnts[2] == 2:
                xmax_one                = send.color_mask_subject_XMax[2][0]
                xmin_one                = send.color_mask_subject_XMin[2][0]
                xmin_two                = send.color_mask_subject_XMin[2][1]
                xmax_two                = send.color_mask_subject_XMax[2][1]
                self.blue_rightside     = max(xmin_one, xmin_two)
                self.blue_leftside      = min(xmax_one, xmax_two)

        else : 
            self.at_reddoor_flag = False
    #----------------Blue_DeepMatrix-----------------
            self.b_y_max        = send.color_mask_subject_YMax[2][0]
            self.b_deep_y       = min(deep.ba)
            self.b_deep_sum     = sum(deep.ba)
            self.b_left_deep    = deep.ba[2]
            self.b_right_deep   = deep.ba[30]
            self.b_center_deep  = deep.ba[16]
    #----------------Y_line_DeepMatrix---------------
            self.y_deep_y           = min(deep.ya)
            self.y_deep_sum         = sum(deep.ya)
            self.y_left_deep        = deep.ya[2]
            self.y_right_deep       = deep.ya[30]
            self.yellow_center_deep = deep.ya[16]
            self.y_deep_left_sum    = sum(deep.ya[0:15])
            self.y_deep_right_sum   = sum(deep.ya[16:31])
    #----------------Filter_matrix-------------------
            filter_matrix          = [max(0, a - b) for a, b in zip(FOCUS_MATRIX, deep.aa)] 
            x_center_num           = sum(i for i, num in enumerate(FOCUS_MATRIX - np.array(deep.aa)) if num >= 0)
            x_center_cnt           = np.sum(np.array(FOCUS_MATRIX) - np.array(deep.aa) >= 0) 
            x_center               = (x_center_num / x_center_cnt) if x_center_cnt > 0 else 0
            left_weight_matrix     = list(range(32))            #0~31
            right_weight_matrix    = list(range(31,-1,-1))      #31~0
            right_weight           = np.dot(filter_matrix,  right_weight_matrix)#內積
            left_weight            = np.dot(filter_matrix,  left_weight_matrix)
            self.deep_y                 = min(deep.aa)
            self.deep_sum               = sum(deep.aa)
            self.deep_sum_l               = sum(deep.aa[0:16])
            self.deep_sum_r              = sum(deep.aa[17:32])
            self.left_deep              = deep.aa[4]
            self.right_deep             = deep.aa[28]
            self.center_deep            = deep.aa[16]
            x_boundary             = 31 if left_weight > right_weight else 0

            if send.color_mask_subject_cnts[1] == 2 and send.color_mask_subject_YMax[0][1] > 100 and send.color_mask_subject_YMax[1][1] > 100: #yellow yellow 值越大 越不容易直走
                self.deep_x = 0
                if self.y_deep_left_sum < self.y_deep_right_sum:
                    self.line_at_right = False
                    self.line_at_left  = True
                elif self.y_deep_left_sum >= self.y_deep_right_sum:
                    self.line_at_right = True
                    self.line_at_left  = False
            else:    
                self.deep_x = x_center - x_boundary
            
class Obs:
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
        self.image.calculate()
        if not self.first_reddoor:
            self.first_reddoor = True
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT + 150,100)
            time.sleep(0.2)
            send.sendContinuousValue(0, 0 , 0 , 0 , 0)
        self.image.calculate()
        while self.image.red_y_max > 140 : #red door 前後修正1 值越大離門越近
            self.image.calculate()
            self.walk.move('small_back')

        if abs(deep.slope) > 0.03 :     # red door 修斜率
            while abs(deep.slope) > 0.03 :
                # self.walk.move('slope_fix')
                self.walk.move('imu_fix')
            # self.need_fix_slope = False
        self.image.calculate()
        if (self.image.red_y_max < 80) :     #red door 前後修正2 值越大離門越近 
            while self.image.red_y_max < 80 :
                self.image.calculate()
                self.walk.move('small_forward')
        elif (self.image.red_y_max > 80) or self.image.b_center_deep == 0:   #red door 前後修正2 值越大離門越近 
            while self.image.red_y_max > 80 or self.image.b_center_deep == 0:
                self.image.calculate()
                self.walk.move('small_back')
        # self.redoor_distence = True
        while 1 :     
            self.image.calculate()
            if (self.image.red_x_min < 2 and self.image.red_x_max > 315) and send.color_mask_subject_size[5][0] > 5000:
                self.image.calculate()
                if (self.image.b_x_max == 0 and self.image.b_x_min == 0 and self.image.blue_leftside <= 20 and self.blue_at_right ) or (self.image.b_x_max == 0 and self.image.b_x_min == 0 and self.image.blue_rightside > 260 and self.blue_at_left ) or (self.image.b_x_max == 0 and self.image.b_x_min == 0 and self.image.blue_rightside == 0 and self.image.blue_leftside == 0):
                    while abs(deep.slope) > 0.03 :
                        self.walk.move('slope_fix')
                    self.crawl()
                    break
                elif (self.image.b_x_min < 2 and self.image.b_x_max > 50):
                    self.image.calculate()
                    self.blue_at_right = True
                    self.blue_at_left = False
                    self.walk.move('slope_right_translate')
                elif (self.image.b_x_max > 315 and self.image.b_x_min < 265):
                    self.image.calculate()
                    self.blue_at_left = True
                    self.blue_at_right = False
                    self.walk.move('slope_left_translate')
                else :
                    self.image.calculate()
                    if self.blue_at_right :
                        self.walk.move('slope_right_translate')
                    elif self.blue_at_left :
                        self.walk.move('slope_left_translate')
                self.image.calculate()
            elif (self.image.red_x_min < 2 and self.image.red_x_max < 315) and send.color_mask_subject_size[5][0] > 5000: 
                self.image.calculate()
                self.walk.move('slope_left_translate')

            elif (self.image.red_x_min > 2 and self.image.red_x_max > 315) and send.color_mask_subject_size[5][0] > 5000: 
                self.image.calculate()
                self.walk.move('slope_right_translate') 
            else :
                self.image.calculate()
                self.walk.move('stay')
                time.sleep(1)
                send.sendHeadMotor(1,1517,180)
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
                send.sendHeadMotor(1,2599,180)
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
                send.sendHeadMotor(1,2048,180)
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
                elif self.door_at_left > 160:
                    while self.image.red_x_max:
                        self.image.calculate()
                        self.walk.move('imu_left_translate')

    def crawl(self):
        self.image.calculate()
        while 1 :
            self.image.calculate()
            if (self.image.b_x_max == 0 and self.image.b_x_min == 0 and self.image.blue_leftside <= 20 and self.blue_at_right ) or (self.image.b_x_max == 0 and self.image.b_x_min == 0 and self.image.blue_rightside > 260 and self.blue_at_left ) or (self.image.b_x_max == 0 and self.image.b_x_min == 0 and self.image.blue_rightside == 0 and self.image.blue_leftside == 0):
                while abs(deep.slope) > 0.03 :
                    self.walk.move('slope_fix')
                break
            elif (self.image.b_x_min < 2 and self.image.b_x_max > 50):
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
        if(self.image.red_y_max < CRMIN):            
            while(self.image.red_y_max < CRMIN):          
                self.image.calculate()
                self.walk.slope()
                self.walk.move('small_forward')
        elif(self.image.red_y_max > CRMAX):          
            while(self.image.red_y_max > CRMAX):         
                self.image.calculate()
                self.walk.slope()
                self.walk.move('small_back')  
        while abs(deep.slope) > 0.03:
            self.walk.slope()
            self.walk.move('slope_fix')
        send.sendContinuousValue(0, 0 , 0 , 0 , 0) 
        time.sleep(1)
        send.sendBodyAuto(0,0,0,0,1,0)
        time.sleep(2)  
        send.sendBodySector(222)
        time.sleep(2.2)
        send.sendBodySector(29)
        time.sleep(0.5)
        send.sendBodySector(1111)
        time.sleep(8)
        while self.crawl_cnt < 3:                
            send.sendBodySector(2222)
            time.sleep(3)
            # time.sleep(0.3)
            self.crawl_cnt += 1
        send.color_mask_subject_YMax[1][0] = 0
        send.sendHeadMotor(1,2048,100)
        send.sendHeadMotor(2,2500,100)
        time.sleep(1)
        while self.crawl_cnt < 7:   
            send.sendBodySector(2222)
            time.sleep(3.5)
            self.crawl_cnt += 1               
            self.image.calculate()
            # print("blue_ymax   = ",self.b_y_max)
            if (send.color_mask_subject_YMax[2][0] >= 35 and send.color_mask_subject_size[2][0] > 5000) or (send.color_mask_subject_YMax[1][0] >= 35 and send.color_mask_subject_size[1][0] > 5000):
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
            send.sendBodyAuto(0,0,0,0,1,0) 
        else :
            send.sendBodySector(3333)
            time.sleep(14)
            send.sendBodySector(29)    
            time.sleep(0.5)
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT,100)
            time.sleep(2)
            send.sendBodyAuto(0,0,0,0,1,0)

    def turn_head(self):
        self.walk.move('stay')
        if not self.image.line_at_right and not self.image.line_at_left: 
            time.sleep(1)
            send.sendHeadMotor(1,1517,180)
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            send.sendHeadMotor(1,1517,180)
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            send.sendHeadMotor(1,1517,180)
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            time.sleep(1.5) 
            if send.color_mask_subject_YMax[1][0] > 220 and send.color_mask_subject_size[1][0] > 5000:
                self.line_at_right_single = True
            if send.color_mask_subject_YMax[5][0] > 220 and send.color_mask_subject_size[5][0] > 5000:
                self.door_at_right = True
                self.door_at_left = False
            self.right_deep_sum = sum(deep.aa) #filter_sum_aa
            print(self.line_at_right_single)
            send.sendHeadMotor(1,2599,180)
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            send.sendHeadMotor(1,2599,180)
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            send.sendHeadMotor(1,2599,180)
            send.sendHeadMotor(2,HEAD_HEIGHT+150,180)
            time.sleep(1.5)
            if send.color_mask_subject_YMax[1][0] > 220 and send.color_mask_subject_size[1][0] > 5000:
                self.line_at_left_single = True
            if send.color_mask_subject_YMax[5][0] > 220 and send.color_mask_subject_size[5][0] > 5000:
                self.door_at_left = True
                self.door_at_right = False
            self.left_deep_sum = sum(deep.aa)
            print(self.line_at_left_single)
            send.sendHeadMotor(1,2048,180)
            send.sendHeadMotor(2,HEAD_HEIGHT,180)
            send.sendHeadMotor(1,2048,180)
            send.sendHeadMotor(2,HEAD_HEIGHT,180)
            send.sendHeadMotor(1,2048,180)
            send.sendHeadMotor(2,HEAD_HEIGHT,180)
            time.sleep(0.3)
        else :
            self.right_deep_sum = 0
            self.left_deep_sum = 0
        if (self.right_deep_sum > self.left_deep_sum) or self.image.line_at_left or self.line_at_left_single or self.door_at_right:        #turn head 右轉
            self.image.calculate()
            if (self.image.b_center_deep > 8):                   #turn head 右轉 前後修正 越大越遠
                while ( self.image.b_center_deep > 8):
                    self.image.calculate()
                    self.walk.move('small_forward')
            elif ( self.image.b_center_deep < 7):                 #turn head 右轉 前後修正 越大越遠
                while ( self.image.b_center_deep < 7 ):
                    self.image.calculate()
                    self.walk.move('small_back')
            if abs(self.walk.get_imu()) < 75:                     #turn head 旋轉角度  右轉 越大轉越多
                while abs(self.walk.get_imu()) < 75:
                    self.walk.move('turn_right')
            send.sendHeadMotor(1,2647,100)
            send.sendHeadMotor(2,1550,100) 
            time.sleep(0.5)
            self.image.calculate()
            send.sendContinuousValue(0, 0 , 0 , 0 , 0)             
            while abs(self.image.deep_x) >= 4 and send.color_mask_subject_size[5][0] < 20000:     #turn head 右轉 直走 結束位置 越小站的越外面
                self.image.calculate()
                self.walk.move('face_right_forward')
            send.sendHeadMotor(1,2048,100)
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
                    self.image.calculate()
                    self.walk.move('small_forward') 
            elif ( self.image.b_center_deep < 7 ):                #turn head 左轉 前後修正 越大越遠 
                while ( self.image.b_center_deep < 7 ):
                    self.image.calculate()
                    self.walk.move('small_back') 
            if abs(self.walk.get_imu()) < 70:                           #turn head 旋轉角度  左轉 越大轉越多
                while abs(self.walk.get_imu()) < 70:
                    self.walk.move('turn_left')
            send.sendHeadMotor(1,1447,100)
            send.sendHeadMotor(2,1550,100) 
            time.sleep(0.5)
            self.image.calculate()
            send.sendContinuousValue(0, 0 , 0 , 0 , 0)                         
            while  abs(self.image.deep_x) >= 4 and send.color_mask_subject_size[5][0] < 20000:        #turn head 左轉  直走 結束位置 越小站的越外面
                self.image.calculate()
                self.walk.move('face_left_forward')
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT,100) 
            time.sleep(0.5)
            self.image.calculate()
            if abs(self.walk.get_imu()) > 50:               #turn head 左轉 回正 數字越小越正對
                while abs(self.walk.get_imu()) > 50:    
                    self.walk.move('turn_left_back')
            if self.door_at_left :
                self.red_door()

    def main(self):
        if send.is_start :
            self.image.calculate()
            # print('a=',self.image.b_x_min)
            # print('b=',self.image.b_x_max)
        #=============================strategy=============================
            if not self.start_walking :                        #指撥後初始動作
                self.walk.imu_yaw_ini()
                #================================================
                self.preturn_left = False
                # self.preturn_left = True
                #================================================
                self.preturn_right = False
                # self.preturn_right = True
                #================================================
                send.sendHeadMotor(1,2048,100)
                send.sendHeadMotor(2,HEAD_HEIGHT  ,100)
                send.sendHeadMotor(1,2048,100)
                send.sendHeadMotor(2,HEAD_HEIGHT  ,100)
                send.sendHeadMotor(1,2048,100)
                send.sendHeadMotor(2,HEAD_HEIGHT  ,100)
                time.sleep(0.5)
                send.sendBodyAuto(0,0,0,0,1,0)
                self.start_walking = True
            if self.preturn_left:                      
                while abs(self.walk.get_imu()) < 25:
                    self.walk.move('preturn_left')
                    rospy.loginfo(f'imu =  {self.walk.get_imu()}')
                self.preturn_left = False
            elif self.preturn_right:                        
                while abs(self.walk.get_imu()) < 50:
                    self.walk.move('preturn_right')
                    rospy.loginfo(f'imu =  {self.walk.get_imu()}')
                self.preturn_right = False
            if self.image.at_reddoor_flag:
                self.red_door()
                pass
            else :
                if self.image.deep_y < 24:
                    self.image.calculate()
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

                    if 13 > self.image.deep_x > 4 :        #normal turn 右轉 範圍越大越容易旋轉 三個地方要調整 大的數字不動
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
                            self.walk.move('turn_right')

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
                            self.walk.move('turn_left')

                        if abs(self.walk.get_imu()) <= 2 :
                            self.imu_ok = True
                    elif (self.image.deep_x < 17 and self.image.deep_x >= 13) or (self.image.deep_x <= -13 and self.image.deep_x > -17) :
                        if (self.image.b_y_max >= 170) and ( abs(self.walk.get_imu()) <= 5 ) and (self.need_imu_back) and (abs(self.image.deep_x) > 3) and (self.image.center_deep != 24) :        #離障礙物太近-->後退
                            while (self.image.b_y_max >= 170):
                                self.image.calculate()
                                self.walk.move('small_back') 
                                if self.image.b_y_max > 0 :
                                    break
                            self.need_imu_back = False
                        if ( abs(self.walk.get_imu()) > 2) and (not self.imu_ok) :                   #IMU修正
                            while ( abs(self.walk.get_imu()) > 2) and (not self.imu_ok) :
                                self.walk.move('imu_fix')
                                self.image.calculate()
                                if abs(self.walk.get_imu()) < 2:        #轉頭策略
                                    if ( self.image.left_deep < 15 ) and ( self.image.right_deep < 15 ) and ( self.image.center_deep < 15 ):
                                        self.turn_head()
                                        self.imu_ok = True
                                        break
                                    elif  self.image.deep_sum_l >= self.image.deep_sum_r :
                                        while abs(self.walk.get_imu()) < 35:    
                                            self.walk.move('turn_left')
                                        self.imu_ok = True
                                    elif  self.image.deep_sum_l < self.image.deep_sum_r :
                                        while abs(self.walk.get_imu()) < 35:    
                                            self.walk.move('turn_right')
                                        self.imu_ok = True
                                else:
                                    pass
                        elif (abs(self.walk.get_imu()) < 2) and (self.imu_ok):
                            if ( self.image.left_deep < 15 ) and ( self.image.right_deep < 15 ) and ( self.image.center_deep < 15 ):
                                self.turn_head()
                                self.imu_ok = True
                            elif  self.image.deep_sum_l >= self.image.deep_sum_r :
                                while abs(self.walk.get_imu()) < 35:    
                                    self.walk.move('turn_left')
                                self.imu_ok = True
                            elif  self.image.deep_sum_l < self.image.deep_sum_r :
                                while abs(self.walk.get_imu()) < 35:    
                                    self.walk.move('turn_right')
                                self.imu_ok = True

                    elif (4 >= self.image.deep_x >= -4) or (abs(self.image.deep_x) >= 17):                  #normal turn 直走 跟一般旋轉值要相等
                        self.walk.move('max_speed')
                        if self.image.deep_x == 0 :
                            self.imu_ok = False
                            self.need_imu_back = True
                            
                elif self.image.deep_y == 24:
                    self.walk.move('max_speed')
                    
        if not send.is_start :
            send.sendSensorReset(1,1,1)
            # # self.image.calculate()
            print("blue_ymax   = ",send.color_mask_subject_YMax[2][0])
            # print("yellow_ymax = ",send.color_mask_subject_YMax[1][0])
            print('ready')
            if self.start_walking :
                send.sendContinuousValue(0,0,0,0,0)
                send.sendBodyAuto(0,0,0,0,1,0)
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
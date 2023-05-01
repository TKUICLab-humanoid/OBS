#!/usr/bin/env python
#coding=utf-8
# from turtle import st
import rospy
import numpy as np
from hello1 import Sendmessage
from ddd import Deep_Calculate
# from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tku_msgs.msg import camera
import cv2 
import sys
import time
import math


#   很多判斷後的動作基本都一樣,可能要討論要怎再整合 可能也會縮短行數
#   image 參數輸出
#   ddd 參數整理


deep            = Deep_Calculate()
send            = Sendmessage()
HEAD_HEIGHT     = 2680
FOCUS_MATRIX    = [2, 2, 2, 2, 6, 6, 7, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 7, 6, 6, 2, 2, 2, 2]
MAX_FORWARD_X         =  4000
MAX_FORWARD_Y         =  -600
MAX_FORWARD_THETA     =     1 
TURN_RIGHT_X            = -1100
TURN_RIGHT_Y            =   1100
TURN_RIGHT_THETA        =    -8
TURN_LEFT_X             = -1500
TURN_LEFT_Y             = -2400
TURN_LEFT_THETA         =     9
# REDDOOR_MOVE_RIGHT      = -2400
# REDDOOR_MOVE_LEFT       =  2400                                                     

class Walk():
    def __init__(self):
        self.image = Normal_Obs_Parameter()  

    def move(self, action_id, z=0, sensor= 0):
        self.image.calculate()
        # imu_flag = self.get_imu() < 0 
        # slope_x_fix             =  100 if self.image.red_y_max  <  150 else -100 if self.image.red_y_max  >  200 else 0
        reddoor_x_fix           =  500 if self.image.b_y_max <  215 else -500 if self.image.b_y_max >  225 else 0
        right_straight_y        = -400 if self.image.center_deep  <  5 else  400 if (self.image.center_deep > 3) and  (self.image.center_deep != 24) else 0                  
        left_straight_y         = 200 if self.image.center_deep <  5 else  -400 if (self.image.center_deep > 3) and  (self.image.center_deep != 24)  else 0                  
        straight_90degree_fix   =  -3   if ((self.get_imu() < 0 and abs(self.get_imu()) < 87) or (self.get_imu() > 0 and abs(self.get_imu()) > 87)) else 3             
        turn_x                  =   self.straight_speed()*2 if self.image.yellow_center_deep < 12 else self.straight_speed()
        # turn_direction_x        =   TURN_RIGHT_X if self.get_imu() > 0 else TURN_LEFT_X  # fix_angle for turn_x
        # turn_direction_y        =   TURN_RIGHT_X if self.get_imu() > 0 else TURN_LEFT_X  # fix_angle for turn_x
        actions             = { 'stay'                  : {'x':  -1200,                  'y':   -600,                 'theta': 0                },
                                'max_speed'             : {'x':  MAX_FORWARD_X,         'y':   MAX_FORWARD_Y,        'theta': MAX_FORWARD_THETA },
                                'small_back'            : {'x': -2500,                  'y':   -400,                 'theta':  0                },
                                'small_forward'         : {'x': 1500,                   'y':  -600,                    'theta': 1 },
                                'turn'                  : {'x': turn_x,  'y': TURN_RIGHT_Y if self.image.deep_x > 0 else TURN_LEFT_Y, 'theta': self.turn_angle() },
                                'imu_fix'               : {'x': TURN_RIGHT_X if self.get_imu() > 0 else TURN_LEFT_X,  'y': TURN_RIGHT_Y if self.get_imu() > 0 else TURN_LEFT_Y, 'theta': self.imu_angle()  },
                                'slope_fix'             : {'x': TURN_LEFT_X if deep.slope > 0 else TURN_RIGHT_X,      'y': TURN_LEFT_Y if deep.slope > 0 else TURN_RIGHT_Y,     'theta': self.slope()      },
                                # 'slope_right_translate' : {'x': 600 + slope_x_fix,      'y': -1200,                 'theta': self.slope()      },
                                # 'slope_left_translate'  : {'x': slope_x_fix,            'y':  1800,                 'theta': self.slope()      },
                                'turn_right'            : {'x': TURN_RIGHT_X,           'y':  TURN_RIGHT_Y,         'theta': TURN_RIGHT_THETA  },
                                'turn_right_back'       : {'x': TURN_LEFT_X,            'y':  TURN_LEFT_Y,          'theta': TURN_LEFT_THETA   },
                                'turn_left'             : {'x': TURN_LEFT_X,            'y':  TURN_LEFT_Y,          'theta': TURN_LEFT_THETA   },
                                'turn_left_back'        : {'x': TURN_RIGHT_X,           'y':  TURN_RIGHT_Y,         'theta': TURN_RIGHT_THETA  },
                                'face_right_forward'    : {'x': MAX_FORWARD_X,          'y':  MAX_FORWARD_Y + right_straight_y ,   'theta': MAX_FORWARD_THETA + straight_90degree_fix    },
                                # 'right_right'         : {'x': SMALL_FORWARD_X,        'y':  SMALL_FORWARD_Y + straight_y_fix,     'theta': SMALL_FORWARD_THETA + straight_90degree_fix    },
                                'face_left_forward'     : {'x': MAX_FORWARD_X,          'y':  MAX_FORWARD_Y + left_straight_y,     'theta': MAX_FORWARD_THETA + straight_90degree_fix   },
                                # 'left_left'           : {'x': SMALL_FORWARD_X,        'y':  SMALL_FORWARD_Y + straight_y_fix,     'theta': SMALL_FORWARD_THETA + straight_90degree_fix    },
                                'preturn_left'          : {'x': TURN_LEFT_X,            'y':  TURN_LEFT_Y,          'theta': TURN_LEFT_THETA   },
                                'preturn_right'         : {'x': TURN_RIGHT_X,           'y':  TURN_RIGHT_Y,         'theta': TURN_RIGHT_THETA  },
                                'reddoor_right_move'    : {'x': -1000 + reddoor_x_fix,  'y':  -2400,   'theta': 0 + self.imu_angle()  },
                                'reddoor_left_move'     : {'x': -1000 + reddoor_x_fix,  'y':  2400,    'theta': 0 + self.imu_angle()  }}
        action              = actions.get(action_id,None)
        if action is not None:
            x              = action['x']
            y              = action['y']
            theta          = action['theta']
            send.sendContinuousValue(x, y, z, theta, sensor)
        # rospy.loginfo(f'walk status =  {action_id}')
    
    def imu_yaw_ini(self):
        self.imu_yaw = 0

    def get_imu(self):
        self.imu_yaw = send.imu_value_Yaw
        return self.imu_yaw 

    def turn_angle(self):
        self.image.calculate()
        turn_ranges = [ (17, -9), 
                        (12, -9), 
                        (8,  -9), 
                        (6,  -7), 
                        (4,  -6), 
                        (2,  -4),  
                        (0,   0),
                        (-2,  3),
                        (-3,  4),
                        (-4,  5),
                        (-6,  7),
                        (-8,  8),
                        (-12, 9),
                        (-17, 9)]
        for turn_range in turn_ranges:
            if  self.image.deep_x >= turn_range[0]:
                return turn_range[1]
        return 0     
    
    def imu_angle(self):      
        imu_ranges = [  (90,   -9), 
                        (60,   -8), 
                        (45,   -7), 
                        (20,   -6), 
                        (10,   -6), 
                        ( 5,   -6), 
                        ( 2,   -5), 
                        ( 0,    0),
                        (-2,    4),
                        (-5,    5),
                        (-10,   5),
                        (-20,   6),
                        (-45,   7),
                        (-60,   8),
                        (-90,   9)]
        for imu_range in imu_ranges:
            if self.imu_yaw >= imu_range[0]:
                return imu_range[1]
        return 0

    def slope(self):
        #-------------------fix to l---------------------
        if deep.slope > 0:          
            slopel_ranges = [(2,     0), 
                             (1,     2), 
                             (0.3,   1), 
                             (0.2,   0), 
                             (0.15,  0), 
                             (0.1,   0), 
                             (0.05,  0), 
                             (0.03,  0), 
                             (0,     0)]
            for slopel_range in slopel_ranges:
                if deep.slope >= slopel_range[0]:
                    return slopel_range[1]
            return 0
        #--------------------fix to r--------------------
        elif deep.slope <= 0:     
            slopel_ranges = [(-2,      0), 
                             (-1,      2), 
                             (-0.3,    1), 
                             (-0.2,    0), 
                             (-0.15,   0), 
                             (-0.1,    0), 
                             (-0.05,   0), 
                             (-0.03,   0), 
                             (0,       0)]
            for slopel_range in slopel_ranges:
                if deep.slope >= slopel_range[0]:
                    return slopel_range[1]
            return 0 
        if send.color_mask_subject_size[5][0] == 0 :
            slope_angle = 0
        return slope_angle

    def straight_speed(self):
        self.image.calculate()
        speed_ranges = [(24,    4000), 
                        (20,    3000), 
                        (16,    2500), 
                        (14,    1800), 
                        (12,    1400), 
                        (8,      800), 
                        (6,      700), 
                        (3,      500), 
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

    def calculate(self):
        self.red_y_max = send.color_mask_subject_YMax[5][0]
        if send.color_mask_subject_size[5][0] > 5000:                      #有紅時計算紅門資訊
            self.at_reddoor_flag = True
            self.red_x_min = send.color_mask_subject_XMin[5][0] 
            self.red_x_max = send.color_mask_subject_XMax[5][0]

            if send.color_mask_subject_cnts[2] == 1:
                self.b_x_min = send.color_mask_subject_XMin[2][0]
                self.b_x_max = send.color_mask_subject_XMax[2][0]
            elif send.color_mask_subject_cnts[2] == 2:
                xmax_one           = send.color_mask_subject_XMax[2][0]
                xmin_one           = send.color_mask_subject_XMin[2][0]
                xmin_two           = send.color_mask_subject_XMin[2][1]
                xmax_two           = send.color_mask_subject_XMax[2][1]
                self.blue_rightside     = max(xmin_one, xmin_two)
                self.blue_leftside      = min(xmax_one, xmax_two)
        else : 
            self.at_reddoor_flag = False
        #----------------Blue_DeepMatrix-----------------
            self.b_y_max        = send.color_mask_subject_YMax[2][0]
            self.b_deep_y       = min(deep.blue_deep)
            self.b_deep_sum     = sum(deep.blue_deep)
            self.b_left_deep    = deep.blue_deep[2]
            self.b_right_deep   = deep.blue_deep[30]
            self.b_center_deep  = deep.blue_deep[16]
            self.b_left_center_deep  = deep.blue_deep[8]
            self.b_right_center_deep  = deep.blue_deep[24]
        #----------------Y_line_DeepMatrix---------------
            self.y_deep_y           = min(deep.yellow_deep)
            self.y_deep_sum         = sum(deep.yellow_deep)
            self.y_left_deep        = deep.yellow_deep[2]
            self.y_right_deep       = deep.yellow_deep[30]
            self.yellow_center_deep = deep.yellow_deep[16]
            self.y_deep_left_sum    = sum(deep.yellow_deep[0:15])
            self.y_deep_right_sum   = sum(deep.yellow_deep[16:31])
            # if send.color_mask_subject_cnts[1] == 1:
                # self.y_dis              = abs(send.color_mask_subject_XMax[0][1] - send.color_mask_subject_XMin[0][1])
        #----------------Filter_matrix-------------------
            filter_matrix          = [max(0, a - b) for a, b in zip(FOCUS_MATRIX, deep.adult)]
            x_center_num           = sum(i for i, num in enumerate(FOCUS_MATRIX - np.array(deep.adult)) if num >= 0)
            x_center_cnt           = np.sum(np.array(FOCUS_MATRIX) - np.array(deep.adult) >= 0)
            x_center               = (x_center_num / x_center_cnt) if x_center_cnt > 0 else 0
            left_weight_matrix     = list(range(32))
            right_weight_matrix    = list(range(31,-1,-1))
            right_weight           = np.dot(filter_matrix,  right_weight_matrix)
            left_weight            = np.dot(filter_matrix,  left_weight_matrix)
            self.deep_y                 = min(deep.adult)
            self.deep_sum               = sum(deep.adult)
            self.left_deep              = deep.adult[4]
            self.right_deep             = deep.adult[28]
            self.center_deep            = deep.adult[16]
            # x_boundary             = 31 if left_weight > right_weight else 0
            if left_weight > right_weight:
                x_boundary = 31
                self.deep_x = x_center - x_boundary
                if abs(self.deep_x) > 16:
                    x_boundary = 0
            elif left_weight <= right_weight:
                x_boundary = 0
                self.deep_x = x_center - x_boundary
                if abs(self.deep_x) > 16:
                    x_boundary = 31

            if send.color_mask_subject_cnts[1] >= 2 and send.color_mask_subject_YMax[1][0] > 235 and send.color_mask_subject_YMax[1][1] > 235 :
                self.deep_x = 0
                if self.y_deep_left_sum < self.y_deep_right_sum:
                    self.line_at_right = False
                    self.line_at_left  = True
                elif self.y_deep_left_sum >= self.y_deep_right_sum:
                    self.line_at_right = True
                    self.line_at_left  = False
            else:    
                self.deep_x = x_center - x_boundary
            # rospy.loginfo(f'right_weight_matrix =  {right_weight_matrix}')
            # rospy.loginfo(f'left_weight_matrix =  {left_weight_matrix}')
            # rospy.loginfo(f'filter =  {filter_matrix}')
            # rospy.loginfo(f'xb =  {x_boundary}')
            # rospy.loginfo(f'xc =  {x_center}')
            # rospy.loginfo(f'right_weight =  {right_weight}')
            # rospy.loginfo(f'left_weight =  {left_weight}')
            # rospy.loginfo(f'x_center_cnt =  {x_center_cnt}')
            # rospy.loginfo(f'x_center_num =  {x_center_num}')
            # rospy.loginfo(f'deep_y =  {self.deep_y}')
            # rospy.loginfo(f'deep_x =  {self.deep_x}')
            # rospy.loginfo(f'b_left_deep =  {self.b_left_deep}')
            # rospy.loginfo(f'b_center_deep =  {self.b_center_deep}')
            rospy.loginfo(f'rrrrrrrrrrrrrrr =  {self.line_at_right}')
            rospy.loginfo(f'llllllllllllll =  {self.line_at_left}')


class Obs:
    def __init__(self):
        self.image          = Normal_Obs_Parameter()
        self.walk           = Walk()
        self.blue_at_left           = False
        self.blue_at_right          = False
        self.redoor_distence        = False
        self.need_fix_slope         = False
        self.first_reddoor          = True
        self.start_walking          = False
        self.imu_ok                 = False
        self.need_imu_back          = True

    def red_door(self):
        if self.first_reddoor:
            self.first_reddoor = False
            self.walk.move('stay')
        
        elif not self.first_reddoor :
            if (abs(self.walk.get_imu()) > 3) and (self.imu_ok == False) :       #IMU修正
                self.walk.move('imu_fix')
            if abs(deep.slope) <= 0.15 :
                if send.color_mask_subject_YMax[2][0] < 190:        #靠近障礙物
                    while send.color_mask_subject_YMax[2][0] < 190:
                        self.walk.move('small_forward') 
                elif send.color_mask_subject_YMax[2][0] >= 190:        #遠離障礙物
                    while send.color_mask_subject_YMax[2][0] >= 190:
                        self.walk.move('small_back') 
                if send.DIOValue == 26 :
                    while abs(self.image.deep_x) > 7:
                        self.walk.move('reddoor_right_move')
                        
                elif send.DIOValue == 25 :
                    while abs(self.image.deep_x) > 7:
                        self.walk.move('reddoor_left_move')
            else :
                #進一般避障
                pass

    def Turn_Head(self):
        self.walk.move('stay')
        if not self.image.line_at_right and not self.image.line_at_left: 
            time.sleep(1)
            send.sendHeadMotor(1, 1497, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT - 80, 100)
            send.sendHeadMotor(1, 1497, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT - 80, 100)
            send.sendHeadMotor(1,1497,100)
            send.sendHeadMotor(2, HEAD_HEIGHT - 80, 100)
            time.sleep(1.8) 
            self.image.calculate()
            right_deep_sum = self.image.deep_sum
            send.sendHeadMotor(1, 2599, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT - 80, 100)
            send.sendHeadMotor(1, 2599, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT - 80, 100)
            send.sendHeadMotor(1, 2599, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT - 80, 100)
            time.sleep(1.8)
            self.image.calculate()
            left_deep_sum = self.image.deep_sum
            send.sendHeadMotor(1, 2048, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT - 80, 100)
            send.sendHeadMotor(1, 2048, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT - 80, 100)
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2, HEAD_HEIGHT - 80, 100)
            time.sleep(1)
            self.image.calculate()

        else :
            right_deep_sum = 0
            left_deep_sum  = 0
        if (right_deep_sum > left_deep_sum) or (self.image.line_at_left) :        #右轉
            self.image.calculate()
            # if self.image.b_y_max < 230 :    
            # while self.image.b_left_deep > 0 and self.image.b_center_deep > 0 and self.image.b_right_deep > 0 :
            #     self.walk.move('small_back')
            #     self.image.calculate()            
            while self.image.b_deep_y > 4:
                self.walk.move('small_forward')
                self.image.calculate()
            # elif self.image.b_y_max > 240 :                
            # if abs(self.walk.get_imu()) < 70:                   #靠近後右旋轉至90度
            while abs(self.walk.get_imu()) < 70:
                self.walk.move('turn_right')
            send.sendHeadMotor(1,2647,100)
            send.sendHeadMotor(2, HEAD_HEIGHT - 80, 100) 
            time.sleep(1)
            self.image.calculate()
            # self.walk.move('stay')
            # if abs(self.image.deep_x) >= 1 :               #直走且imu修正
            while abs(self.image.deep_x) >= 8 :     #轉頭後直走 平移修正
                # if abs(self.walk.get_imu()) > 87 :          #視步態更動
                #     if self.image.left_deep != 24 :
                self.walk.move('face_right_forward')
                self.image.calculate()
                    # else :
                    #     break
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,2600,100) 
            time.sleep(1)
            self.image.calculate()
            # if abs(self.walk.get_imu()) > 45:             #右轉回正
            while abs(self.walk.get_imu()) > 45:
                self.walk.move('turn_right_back')
        elif (left_deep_sum > right_deep_sum) or (self.image.line_at_right) :         #左轉
            self.image.calculate()
            # if self.image.b_y_max < 230 :    
            # while self.image.b_left_deep < 0 and self.image.b_center_deep < 0 and self.image.b_right_deep < 0  :
            #     self.walk.move('small_back')
            #     self.image.calculate()               
            while self.image.b_deep_y > 4:
                self.walk.move('small_forward') 
                self.image.calculate()
            # elif self.image.b_y_max > 240 :                
            # if abs(self.walk.get_imu()) < 65 :                           #靠近後轉至90度
            while abs(self.walk.get_imu()) < 65 :
                self.walk.move('turn_left')
                self.image.calculate()
            send.sendHeadMotor(1,1447,100)
            send.sendHeadMotor(2,HEAD_HEIGHT - 80,100) 
            time.sleep(1)
            self.image.calculate()
            # self.walk.move('stay')
            # if  abs(self.image.deep_x) >= 1 :                          #直走且imu修正
            while  abs(self.image.deep_x) >= 8 :
                # if abs(self.walk.get_imu()) > 87:
                # if self.image.right_deep != 24 :
                self.walk.move('face_left_forward')
                self.image.calculate()
                # else :
                    # break
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT - 80,100) 
            time.sleep(1)
            self.image.calculate()
            # if abs(self.walk.get_imu()) > 50:               #左轉回正
            while abs(self.walk.get_imu()) > 50:    
                self.walk.move('turn_left_back')

    def main(self):
        if send.is_start :
        #=============================strategy=============================
            if not self.start_walking :                        #指撥後初始動作
                self.walk.imu_yaw_ini()
                self.preturn_left = False
                # self.preturn_left = True
                self.preturn_right = False
                # self.preturn_right = True
                send.sendHeadMotor(1,2048,100)
                send.sendHeadMotor(2,HEAD_HEIGHT,100)
                time.sleep(0.5)
                send.sendBodyAuto(0,0,0,0,1,0)
                self.start_walking = True
            if self.preturn_left:                      #指定初始向左旋轉
                while abs(self.walk.get_imu()) < 50:
                    self.walk.move('preturn_left')
                    rospy.loginfo(f'imu =  {self.walk.get_imu()}')
                self.preturn_left = False
            elif self.preturn_right:                        #指定初始向右旋轉 
                while abs(self.walk.get_imu()) < 45:
                    self.walk.move('preturn_right')
                    rospy.loginfo(f'imu =  {self.walk.get_imu()}')
                self.preturn_right = False
            self.image.calculate()
            if self.image.deep_y < 24:
                self.image.calculate()
                #黃線判斷
                if self.image.line_at_right :
                    if self.image.y_deep_left_sum > self.image.y_deep_right_sum :
                        self.line_at_right = True
                    elif (self.image.y_deep_left_sum < self.image.y_deep_right_sum) or (self.image.y_deep_right_sum > 350) :
                        self.image.line_at_right = False
                elif self.image.line_at_left :
                    if self.image.y_deep_left_sum < self.image.y_deep_right_sum :
                        self.image.line_at_left = True
                    elif (self.image.y_deep_left_sum > self.image.y_deep_right_sum) or (self.image.y_deep_right_sum > 350) :
                        self.image.line_at_left = False
                #進紅門
                if (self.image.red_y_max >= 100) and (self.image.at_reddoor_flag):
                    self.red_door()
                #一般往右避障
                if 13 > self.image.deep_x > 9 :        
                    self.walk.straight_speed()
                    if ((abs(self.walk.get_imu()) > 5) and (not self.imu_ok)) and (self.image.deep_x >= 7) :       #IMU修正
                        self.walk.move('imu_fix')
                    else:
                        self.walk.move('turn')

                    if abs(self.walk.get_imu()) <= 5 :
                        self.imu_ok = True
                #一般往左避障
                elif -9 > self.image.deep_x > -13 :   
                    self.walk.straight_speed()
                    if ((abs(self.walk.get_imu()) > 5) and (not self.imu_ok)) and (self.image.deep_x <= -7) :      #IMU修正
                        self.walk.move('imu_fix')
                    else:
                        self.walk.move('turn')

                    if abs(self.walk.get_imu()) <= 5 :
                        self.imu_ok = True
                #一般轉頭避障
                elif (self.image.deep_x < 17 and self.image.deep_x >= 13) or (self.image.deep_x <= -13 and self.image.deep_x > -17) :
                    if  abs(self.walk.get_imu()) > 5 :
                        while  abs(self.walk.get_imu()) > 5 :
                            self.walk.move('imu_fix')
                    # elif abs(self.walk.get_imu()) < 5:
                        # if (self.image.b_y_max >= 200) and (self.need_imu_back) and (self.image.center_deep != 24) :        #離障礙物太近-->後退
                        #     while self.image.b_y_max >= 200 :
                        #         self.image.calculate()
                        #         self.walk.move('small_back') 
                        #     self.need_imu_back = False
                    self.image.calculate()
                    if (( self.image.left_deep < 14 ) and ( self.image.right_deep < 14 ) and ( self.image.center_deep < 14 )) or ((self.image.line_at_left or self.image.line_at_right) and self.image.center_deep < 14):
                        self.Turn_Head()
                        # self.imu_ok = True
                    elif  self.image.deep_x < 0 : 
                        while abs(self.image.deep_x) > 9 :
                            self.walk.move('turn_left')
                            self.image.calculate()
                        # if  self.image.y_deep_y < 12 and self.image.y_deep_left_sum < self.image.y_deep_left_sum :
                        #     if self.image.b_left_center_deep < 12 :
                        #         while abs(self.walk.get_imu()) < 30 :
                        #             self.walk.move('turn_right')
                        #         # self.imu_ok = True
                        #     else :
                        #         while abs(self.walk.get_imu()) < 30 :
                        #             self.walk.move('turn_left')
                        #         # self.imu_ok = True
                            
                        # else :
                        #     while abs(self.walk.get_imu()) < 30 :
                        #         self.walk.move('turn_left')
                            # self.imu_ok = True
                            
                    elif  self.image.deep_x > 0 :
                        while abs(self.image.deep_x) > 9 :
                            self.walk.move('turn_right')
                            self.image.calculate()
                        # if self.image.y_deep_y < 12 and self.image.y_deep_left_sum >= self.image.y_deep_left_sum :
                        #     if self.image.b_right_center_deep < 12 :
                        #         while abs(self.walk.get_imu()) < 30 :
                        #             self.walk.move('turn_left')
                        #         # self.imu_ok = True
                        #     else :
                        #         while abs(self.walk.get_imu()) < 30 :
                        #             self.walk.move('turn_right')
                        #         # self.imu_ok = True
                            
                        # else :
                        #     while abs(self.walk.get_imu()) < 30 :
                        #         self.walk.move('turn_right')
                        #     # self.imu_ok = True
                            
                #不須避障
                elif (9 >= self.image.deep_x >= -9) or (abs(self.image.deep_x) >= 17) :                  #最高速直走
                    self.walk.move('max_speed')
                    if self.image.line_at_left or self.image.line_at_right:
                        self.imu_ok = True
                    else :
                        self.imu_ok = False
                else :
                    pass
            #沒看到障礙物
            elif self.image.deep_y == 24:
                self.walk.move('max_speed')
                    
        if not send.is_start :
            if self.start_walking :
                send.sendContinuousValue(0,0,0,0,0)
                send.sendBodyAuto(0,0,0,0,1,0)
                time.sleep(0.5)
                self.start_walking = False
            # send.sendContinuousValue(0,0,0,0,0)
            # self.walk.move('stay')

if __name__ == '__main__':

    try:
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
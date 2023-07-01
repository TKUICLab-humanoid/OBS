#!/usr/bin/env python
#coding=utf-8
# from turtle import st
import rospy
import numpy as np
from hello1 import Sendmessage
# import sys
# sys.path.append('/home/iclab/Desktop/adult_hurocup/src/strategy')
# from Python_API import Sendmessage
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


deep                  = Deep_Calculate()
send                  = Sendmessage()
HEAD_HEIGHT           = 1436
FOCUS_MATRIX          = [0, 0, 0, 1, 2, 3, 5, 6, 6, 6, 7, 7, 8, 9, 10, 11, 11, 10, 9, 8, 7, 7, 6, 6, 6, 5, 3, 2, 1, 0, 0, 0]
# FOCUS_MATRIX    = [0, 0, 0, 1, 2, 3, 5, 6, 6, 6, 7, 7, 8, 8, 9, 10, 10, 9, 8, 8, 7, 7, 6, 6, 6, 5, 3, 2, 1, 0, 0, 0]
MAX_FORWARD_X         =  5000
MAX_FORWARD_Y         =  0
MAX_FORWARD_THETA     =  0
TURN_RIGHT_X          = -2100
TURN_RIGHT_Y          =  2000
TURN_RIGHT_THETA      =    -8
IMU_RIGHT_X           = -2100 #angle : 5
IMU_RIGHT_Y           =  2000
TURN_LEFT_X           = -1400
TURN_LEFT_Y           = -1800
TURN_LEFT_THETA       =     8
IMU_LEFT_X            = -1800 #angle : 5
IMU_LEFT_Y            = -1000
# REDDOOR_MOVE_RIGHT      = -2400
# REDDOOR_MOVE_LEFT       =  2400                                                     

class Walk():
    def __init__(self):
        self.image = Normal_Obs_Parameter()  
        self.now_speed = 0

    def move(self, action_id, z=0, sensor= 0):
        self.image.calculate()
        if  MAX_FORWARD_X - self.now_speed >= 500:
            self.now_speed += 100 
        else :
            self.now_speed = MAX_FORWARD_X
        # imu_flag = self.get_imu() < 0 
        # slope_x_fix             =  100 if self.image.red_y_max  <  150 else -100 if self.image.red_y_max  >  200 else 0
        reddoor_x_fix           =  1000 if self.image.red_deep_center_y > 7 else -1000 if self.image.red_deep_center_y < 6 else 0  #越遠數字越big
        right_straight_y        = -1500 if self.image.center_deep  <  2 else  2200 if (self.image.center_deep > 2) and  (self.image.center_deep != 24) else 0                  
        left_straight_y         = 1800 if self.image.center_deep <  2 else  -1500 if (self.image.center_deep > 2) and  (self.image.center_deep != 24)  else 0                  
        straight_90degree_fix   =  -3   if ((self.get_imu() < 0 and abs(self.get_imu()) < 90) or (self.get_imu() > 0 and abs(self.get_imu()) > 87)) else 3             
        turn_x                  =   self.straight_speed()  if self.image.y_deep_y < 10 else self.straight_speed()
        # turn_direction_x        =   TURN_RIGHT_X if self.get_imu() > 0 else TURN_LEFT_X  # fix_angle for turn_x
        # turn_direction_y        =   TURN_RIGHT_X if self.get_imu() > 0 else TURN_LEFT_X  # fix_angle for turn_x
        actions             = { 'stay'                  : {'x':  -1500,                  'y':   0,                 'theta': 0               },
                                'max_speed'             : {'x':  self.now_speed,         'y':   MAX_FORWARD_Y,        'theta': MAX_FORWARD_THETA },
                                # 'change_speed'          : {'x': self.straight_speed(),  'y':   MAX_FORWARD_Y,        'theta': MAX_FORWARD_THETA },
                                'small_back'            : {'x': -2500,                  'y':   0,                 'theta':  0                },
                                'small_forward'         : {'x': 3500,                   'y':  0,                    'theta': 0 },
                                'turn'                  : {'x': turn_x,  'y': TURN_RIGHT_Y if self.image.deep_x > 0 else TURN_LEFT_Y, 'theta': self.turn_angle() },
                                'imu_fix'               : {'x': IMU_RIGHT_X if self.get_imu() > 0 else IMU_LEFT_X,  'y': IMU_RIGHT_Y if self.get_imu() > 0 else IMU_LEFT_Y, 'theta': self.imu_angle()  },
                                'slope_fix'             : {'x': TURN_LEFT_X if deep.slope > 0 else TURN_RIGHT_X,      'y': TURN_LEFT_Y if deep.slope > 0 else TURN_RIGHT_Y,     'theta': self.slope()      },
                                # 'slope_right_translate' : {'x': 600 + slope_x_fix,      'y': -1200,                 'theta': self.slope()      },
                                # 'slope_left_translate'  : {'x': slope_x_fix,            'y':  1800,                 'theta': self.slope()      },
                                'turn_right'            : {'x': TURN_RIGHT_X,           'y':  TURN_RIGHT_Y,         'theta': TURN_RIGHT_THETA  },
                                'turn_right_back'       : {'x': TURN_LEFT_X,            'y':  TURN_LEFT_Y,          'theta': TURN_LEFT_THETA   },
                                'turn_left'             : {'x': TURN_LEFT_X,            'y':  TURN_LEFT_Y,          'theta': TURN_LEFT_THETA   },
                                'turn_left_back'        : {'x': TURN_RIGHT_X,           'y':  TURN_RIGHT_Y,         'theta': TURN_RIGHT_THETA  },
                                'face_right_forward'    : {'x': 4000,          'y':  MAX_FORWARD_Y + right_straight_y ,   'theta': MAX_FORWARD_THETA + straight_90degree_fix    },
                                # 'right_right'         : {'x': SMALL_FORWARD_X,        'y':  SMALL_FORWARD_Y + straight_y_fix,     'theta': SMALL_FORWARD_THETA + straight_90degree_fix    },
                                'face_left_forward'     : {'x': 4000,          'y':  MAX_FORWARD_Y + left_straight_y,     'theta': MAX_FORWARD_THETA + straight_90degree_fix   },
                                # 'left_left'           : {'x': SMALL_FORWARD_X,        'y':  SMALL_FORWARD_Y + straight_y_fix,     'theta': SMALL_FORWARD_THETA + straight_90degree_fix    },
                                'preturn_left'          : {'x': TURN_LEFT_X,            'y':  TURN_LEFT_Y,          'theta': TURN_LEFT_THETA   },
                                'preturn_right'         : {'x': TURN_RIGHT_X,           'y':  TURN_RIGHT_Y,         'theta': TURN_RIGHT_THETA  },
                                'reddoor_right_move'    : {'x': 0 + reddoor_x_fix,  'y':  -2200,   'theta': -2 + self.imu_angle()  },
                                'reddoor_left_move'     : {'x': -800 + reddoor_x_fix,  'y':  2600,    'theta': 1 + self.imu_angle()  }}
        action              = actions.get(action_id,None)
        if action is not None:
            x              = action['x']
            y              = action['y']
            theta          = action['theta']
            send.sendContinuousValue(x, y, z, theta, sensor)
            self.now_speed = x
        rospy.loginfo(f'walk status =  {action_id}')
        rospy.loginfo(f'xxxxxxxxxxxxxxxxxxxxxxxxxxxxxx =  {self.now_speed}')
    
    def imu_yaw_ini(self):
        self.imu_yaw = 0

    def get_imu(self):
        self.imu_yaw = send.imu_value_Yaw
        return self.imu_yaw 

    def turn_angle(self):
        self.image.calculate()
        turn_ranges = [ (17, -8), 
                        (12, -8), 
                        (8,  -7), 
                        (6,  -7), 
                        (4,  -5), 
                        (2,  -3),  
                        (0,   0),
                        (-2,  3),
                        (-4,  5),
                        (-6,  7),
                        (-8,  7),
                        (-12, 8),
                        (-17, 8)]
        for turn_range in turn_ranges:
            if  self.image.deep_x >= turn_range[0]:
                return turn_range[1]
        return 0     
    
    def imu_angle(self):      
        imu_ranges = [  (90,   -8), 
                        (60,   -8), 
                        (45,   -8), 
                        (20,   -6), 
                        (10,   -4), 
                        ( 5,   -3), 
                        ( 2,   -2), 
                        ( 0,    0),
                        (-2,    1),
                        (-5,    2),
                        (-10,   3),
                        (-20,   6),
                        (-45,   8),
                        (-60,   8),
                        (-90,   8)]
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
        if self.image.red_size == 0 :
            slope_angle = 0
        return slope_angle

    def straight_speed(self):
        self.image.calculate()
        speed_ranges = [(24,    6000), 
                        (20,    5600), 
                        (16,    5000), 
                        (14,    4300), 
                        (12,    3000), 
                        (8,     2600), 
                        (6,     2000), 
                        (3,     1500), 
                        (0,     600)]
        for speed_range in speed_ranges:
            if self.image.deep_center_y >= speed_range[0]:
                # if  speed_range[1] - self.now_speed >= 500:
                #     return self.now_speed + 500
                # else :
                return speed_range[1]
        return 0

    # def straight_speed(self):
    #     self.image.calculate()
    #     speed_ranges = [(24,    6000), 
    #                     (20,    5000), 
    #                     (16,    4000), 
    #                     (14,    3000), 
    #                     (12,    2600), 
    #                     (8,      2000), 
    #                     (6,      1500), 
    #                     (3,      800), 
    #                     (0,      600)]
    #     for speed_range in speed_ranges:
    #         if self.image.deep_y >= speed_range[0]:
    #             return speed_range[1]
    #     return 0

class Normal_Obs_Parameter:
    def __init__(self):
        self.line_at_left               = False
        self.line_at_right              = False
        self.at_reddoor_flag            = False
        self.deep_y                     = 24
        self.deep_center_y              = 24
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

    def calculate(self):
        self.red_cnt        = send.color_mask_subject_cnts[5]
        self.blue_cnt       = send.color_mask_subject_cnts[2]
        self.yellow_cnt     = send.color_mask_subject_cnts[1]
        self.red_y_max      = max(np.array(send.color_mask_subject_YMax[5])) if self.red_cnt > 0 else 0
        self.red_size       = max(np.array(send.color_mask_subject_size[5])) if self.red_cnt > 0 else 0
        if self.red_size > 5000:   
            self.at_reddoor_flag = True
        else : 
            self.at_reddoor_flag = False
        self.red_x_min              =     np.array(send.color_mask_subject_XMin[5])[0]  if self.red_cnt >= 1 else 0
        self.red_x_max              =     np.array(send.color_mask_subject_XMax[5])[0]  if self.red_cnt >= 1 else 0
        self.blue_rightside         = min(np.array(send.color_mask_subject_XMax[2])) if self.blue_cnt >= 2 else 0
        self.blue_leftside          = max(np.array(send.color_mask_subject_XMin[2])) if self.blue_cnt >= 2 else 0
        self.b_x_min                =     np.array(send.color_mask_subject_XMin[2])  if self.blue_cnt == 1 else 0
        self.b_x_max                =     np.array(send.color_mask_subject_XMax[2])  if self.blue_cnt == 1 else 0
        
        # if send.color_mask_subject_cnts[2] == 1:
        #     self.b_x_min = send.color_mask_subject_XMin[2][0]
        #     self.b_x_max = send.color_mask_subject_XMax[2][0]
        # # elif send.color_mask_subject_cnts[2] == 2:
        #     xmax_one           = send.color_mask_subject_XMax[2][0]
        #     xmin_one           = send.color_mask_subject_XMin[2][0]
        #     xmin_two           = send.color_mask_subject_XMin[2][1]
        #     xmax_two           = send.color_mask_subject_XMax[2][1]
        #     self.blue_rightside     = max(xmin_one, xmin_two)
        #     self.blue_leftside      = min(xmax_one, xmax_two)

        #----------------Blue_DeepMatrix-----------------
        self.b_y_max                = max(np.array(send.color_mask_subject_YMax[2])) if self.blue_cnt >  0 else 0
        self.b_deep_y               = min(np.array(deep.blue_deep))
        self.b_deep_sum             = sum(np.array(deep.blue_deep))
        self.b_left_deep            =     np.array(deep.blue_deep)[2]
        self.b_right_deep           =     np.array(deep.blue_deep)[30]
        self.b_center_deep          =     np.array(deep.blue_deep)[16]
        self.b_left_center_deep     =     np.array(deep.blue_deep)[8]
        self.b_right_center_deep    =     np.array(deep.blue_deep)[24]
        #----------------Y_line_DeepMatrix---------------
        self.y_deep_y               = min(np.array(deep.yellow_deep))
        self.y_deep_sum             = sum(np.array(deep.yellow_deep))
        self.y_left_deep            =     np.array(deep.yellow_deep)[3]
        self.y_right_deep           =     np.array(deep.yellow_deep)[28]
        self.yellow_center_deep     =     np.array(deep.yellow_deep)[16]
        self.y_deep_left_sum        = sum(np.array(deep.yellow_deep)[0:15])
        self.y_deep_right_sum       = sum(np.array(deep.yellow_deep)[16:31])
        self.yellow_Ymax            = max(np.array(send.color_mask_subject_YMax[1]))     if self.yellow_cnt >  0 else 0
        self.first_yellow_Ymax      =     np.array(send.color_mask_subject_YMax[1])[0]   if self.yellow_cnt >= 2 else self.yellow_Ymax
        self.sec_yellow_Ymax        =     np.array(send.color_mask_subject_YMax[1])[1]   if self.yellow_cnt >= 2 else 0
        self.first_yellow_size      = max(np.array(send.color_mask_subject_size[1]))     if self.yellow_cnt >  1 else 0
        
        # if send.color_mask_subject_cnts[1] == 1:
            # self.y_dis              = abs(send.color_mask_subject_XMax[0][1] - send.color_mask_subject_XMin[0][1])
        #----------------Filter_matrix-------------------
        filter_matrix               = [max(0, a - b) for a, b in zip(FOCUS_MATRIX, deep.adult)]
        x_center_num                = sum(i for i, num in enumerate(FOCUS_MATRIX - np.array(deep.adult)) if num >= 0)
        x_center_cnt                =     np.sum(np.array(FOCUS_MATRIX) - np.array(deep.adult) >= 0)
        x_center                    = (x_center_num / x_center_cnt) if x_center_cnt > 0 else 0
        left_weight_matrix          = list(range(32))
        right_weight_matrix         = list(range(31,-1,-1))
        right_weight                =     np.dot(filter_matrix,  right_weight_matrix)
        left_weight                 =     np.dot(filter_matrix,  left_weight_matrix)
        self.deep_y                 = min(np.array(deep.adult))
        self.deep_center_y          = min(np.array(deep.adult)[10:23])
        self.max_deep_center_y      = max(np.array(deep.adult)[10:23])
        self.red_deep_center_y      = min(np.array(deep.adult)[10:23])
        self.deep_sum               = sum(np.array(deep.adult))
        self.deep_sum_l             = sum(np.array(deep.adult)[0:16])
        self.deep_sum_r             = sum(np.array(deep.adult)[17:32])
        self.left_deep              =     np.array(deep.adult)[4]
        self.left_center_deep       =     np.array(deep.adult)[8]
        self.right_deep             =     np.array(deep.adult)[28]
        self.right_center_deep      =     np.array(deep.adult)[24]
        self.center_deep            =     np.array(deep.adult)[16]
        x_boundary                  = 31 if left_weight > right_weight else 0
        if self.yellow_cnt >= 2 and self.first_yellow_Ymax > 225 and self.sec_yellow_Ymax > 225 :
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
        # rospy.loginfo(f'focus =  {FOCUS_MATRIX}')
        # rospy.loginfo(f'deep =  {deep.adult}')
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
        # rospy.loginfo(f'rrrrrrrrrrrrrrr =  {self.line_at_right}')
        # rospy.loginfo(f'llllllllllllll =  {send.color_mask_subject_YMax[2][0]}')
        # print(send.color_mask_subject_YMax[2][0])
        # print(self.deep_center_y)


class Obs:
    def __init__(self):
        self.image                  = Normal_Obs_Parameter()
        self.walk                   = Walk()
        self.redoor_distence        = False
        self.need_fix_slope         = False
        self.first_reddoor          = True
        self.start_walking          = False
        self.imu_ok                 = False
        self.need_imu_back          = True
        self.red_avoid_distance     = False
        
    def red_door(self):
        self.image.calculate()
        while abs(self.walk.get_imu()) > 5 and not self.imu_ok :       #IMU修正
            self.image.calculate()
            self.walk.move('imu_fix')
        self.imu_ok = True
        #-------------------------------------
        while self.image.red_deep_center_y > 6: #越遠數字越bid
            self.image.calculate()
            self.walk.move('small_forward') 
        while self.image.red_deep_center_y < 5:
            self.image.calculate()
            self.walk.move('small_back') 
            
        #-------------------------------------
        # if send.DIOValue == 26 :
        #     self.image.calculate()
        #     while self.image.red_deep_center_y < 15:
        #         self.image.calculate()
        #         self.walk.move('reddoor_right_move')
        #     while abs(self.walk.get_imu()) < 10:
        #         self.walk.move('turn_right')
        if send.DIOValue == 26 :
            while abs(self.walk.get_imu()) < 60:
                self.walk.move('turn_right')
            self.walk.move('stay')
            send.sendHeadMotor(1,2647,100)
            send.sendHeadMotor(2, HEAD_HEIGHT + 120, 100) 
            time.sleep(1)
            self.image.calculate()
            while abs(self.image.red_deep_center_y) < 12 :     #轉頭後直走 平移修正 abs(self.image.deep_x) >= 6
                self.walk.move('face_right_forward')
                self.image.calculate()
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT,100) 
            time.sleep(1)
            self.image.calculate()
            while abs(self.walk.get_imu()) > 50:
                self.walk.move('turn_right_back')
        #-------------------------------------
        # elif send.DIOValue == 25 :
        #     self.image.calculate()
        #     while self.image.red_deep_center_y < 15: #abs(self.image.deep_x) > 13
        #         self.image.calculate()
        #         self.walk.move('reddoor_left_move')
        #     while abs(self.walk.get_imu()) < 10:
        #         self.walk.move('turn_left')
        elif send.DIOValue == 25 :
            while abs(self.walk.get_imu()) < 70 :
                self.walk.move('turn_left')
                self.image.calculate()
            self.walk.move('stay')
            send.sendHeadMotor(1,1449,100)
            send.sendHeadMotor(2,HEAD_HEIGHT + 120,100) 
            time.sleep(1)
            self.image.calculate()
            while  abs(self.image.red_deep_center_y) < 12 :
                self.walk.move('face_left_forward')
                self.image.calculate()
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT ,100) 
            time.sleep(1)
            self.image.calculate()
            while abs(self.walk.get_imu()) > 50 :     # abs(self.walk.get_imu()) > 50
                self.walk.move('turn_left_back')

    def Turn_Head(self):
        self.walk.move('stay')
        if not self.image.line_at_right and not self.image.line_at_left: 
            time.sleep(1)
            send.sendHeadMotor(1, 1550, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT +160, 100) #80
            send.sendHeadMotor(1, 1550, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT +160, 100)
            send.sendHeadMotor(1, 1550, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT +160, 100)
            time.sleep(1.8) 
            self.image.calculate()
            rospy.loginfo(f'dy =  {self.image.max_deep_center_y}')
            right_deep_sum = self.image.deep_sum #self.image.deep_sum
            send.sendHeadMotor(1, 2546, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT +160, 100)
            send.sendHeadMotor(1, 2546, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT +160, 100)
            send.sendHeadMotor(1, 2546, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT +160, 100)
            time.sleep(1.8)
            self.image.calculate()
            rospy.loginfo(f'dy =  {self.image.max_deep_center_y}')
            left_deep_sum = self.image.deep_sum
            send.sendHeadMotor(1, 2048, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT +80, 100)
            send.sendHeadMotor(1, 2048, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT +80, 100)
            send.sendHeadMotor(1, 2048, 100)
            send.sendHeadMotor(2, HEAD_HEIGHT +80, 100)
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
            while self.image.b_deep_y > 2: #more close more small
                self.walk.move('small_forward')
                self.image.calculate()
            while self.image.b_deep_y < 1: #more close more small
                self.walk.move('small_back')
                self.image.calculate()
            # elif self.image.b_y_max > 240 :                
            # if abs(self.walk.get_imu()) < 70:                   #靠近後右旋轉至90度
            while abs(self.walk.get_imu()) < 75:
                self.walk.move('turn_right')
            self.walk.move('stay')
            send.sendHeadMotor(1,2647,100)
            send.sendHeadMotor(2, HEAD_HEIGHT + 120, 100) 
            time.sleep(1)
            self.image.calculate()
            # self.walk.move('stay')
            # if abs(self.image.deep_x) >= 1 :               #直走且imu修正
            while abs(self.image.red_deep_center_y) < 10 :     #轉頭後直走 平移修正 abs(self.image.deep_x) >= 6
                # if abs(self.walk.get_imu()) > 87 :          #視步態更動
                #     if self.image.left_deep != 24 :
                self.walk.move('face_right_forward')
                self.image.calculate()
                    # else :
                    #     break
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT,100) 
            time.sleep(1)
            self.image.calculate()
            # if abs(self.walk.get_imu()) > 45:             #右轉回正
            while abs(self.walk.get_imu()) > 50:
                self.walk.move('turn_right_back')
        elif (left_deep_sum > right_deep_sum) or (self.image.line_at_right) :         #左轉
            self.image.calculate()
            # if self.image.b_y_max < 230 :    
            # while self.image.b_left_deep < 0 and self.image.b_center_deep < 0 and self.image.b_right_deep < 0  :
            #     self.walk.move('small_back')
            #     self.image.calculate()               
            while self.image.b_deep_y > 3:
                self.walk.move('small_forward') 
                self.image.calculate()
            while self.image.b_deep_y < 2: #more close more small
                self.walk.move('small_back')
                self.image.calculate()
            # elif self.image.b_y_max > 240 :                
            # if abs(self.walk.get_imu()) < 65 :                           #靠近後轉至90度
            while abs(self.walk.get_imu()) < 60 :
                self.walk.move('turn_left')
            self.walk.move('stay')
            send.sendHeadMotor(1,1449,100)
            send.sendHeadMotor(2,HEAD_HEIGHT + 120,100) 
            time.sleep(1)
            self.image.calculate()
            # self.walk.move('stay')
            # if  abs(self.image.deep_x) >= 1 :                          #直走且imu修正
            while  abs(self.image.red_deep_center_y) < 10 :
                # if abs(self.walk.get_imu()) > 87:
                # if self.image.right_deep != 24 :
                self.walk.move('face_left_forward')
                self.image.calculate()
                # else :
                    # break
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT ,100) 
            time.sleep(1)
            self.image.calculate()
            # if abs(self.walk.get_imu()) > 50:               #左轉回正
            while abs(self.walk.get_imu()) > 40:    
                self.walk.move('turn_left_back')

    def main(self):
        if send.is_start :
            rospy.loginfo(f'imu 0k  =  {self.imu_ok}')
        #=============================strategy=============================
            self.image.calculate()
            if not self.start_walking :                        #指撥後初始動作
                self.walk.imu_yaw_ini()
                self.preturn_left = False
                # self.preturn_left = True
                self.preturn_right = False
                # self.preturn_right = True
                send.sendHeadMotor(1,2048,100)
                send.sendHeadMotor(2,HEAD_HEIGHT,100)
                time.sleep(0.5)
                # send.sendBodySector(1111)
                # time.sleep(2)
                # send.sendBodySector(1218)
                # time.sleep(1)
                send.sendBodyAuto(0,0,0,0,1,0)
                self.start_walking = True
            if self.preturn_left:                      #指定初始向左旋轉
                while abs(self.walk.get_imu()) < 30:
                    self.walk.move('preturn_left')
                    rospy.loginfo(f'imu =  {self.walk.get_imu()}')
                self.preturn_left = False
            elif self.preturn_right:                        #指定初始向右旋轉 
                while abs(self.walk.get_imu()) < 30:
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
                
                #進紅門
                self.image.calculate()
                if (self.image.red_y_max >= 80) and self.image.at_reddoor_flag and self.first_reddoor: #and (self.first_reddoor)
                    self.imu_ok = False
                    self.red_door()
                    self.first_reddoor = False
                #一般往左右避障
                if (14 > self.image.deep_x > 6) or (-6 > self.image.deep_x > -14) :
                    # while self.image.deep_center_y < 3   : #and self.red_avoid_distance == False
                    #     self.walk.move('small_back')
                    #     self.image.calculate()
                # if (13 > self.image.deep_x > -13) :        
                    self.walk.straight_speed()
                    if ((abs(self.walk.get_imu()) > 5) and (not self.imu_ok)) and (abs(self.image.deep_x) >= 9) :       #IMU修正
                    # if ((abs(self.walk.get_imu()) > 5) and (not self.imu_ok)) :       #IMU修正
                    # if  (not self.imu_ok):       #IMU修正
                        while abs(self.walk.get_imu()) > 5 :
                            self.walk.move('imu_fix')
                    elif (self.image.line_at_left and self.image.center_deep < 14 and self.image.right_deep < 14) or (self.image.line_at_right and self.image.center_deep < 14 and self.image.left_deep < 14):
                        self.Turn_Head()
                    else:
                        self.walk.move('turn')

                    if abs(self.walk.get_imu()) <= 10 :
                        self.imu_ok = True
                #一般往左避障
                # elif -7 > self.image.deep_x > -13 :   
                #     self.walk.straight_speed()
                #     if ((abs(self.walk.get_imu()) > 5) and (not self.imu_ok)) and (self.image.deep_x <= -9) :      #IMU修正
                #         self.walk.move('imu_fix')
                #     else:
                #         self.walk.move('turn')

                #     if abs(self.walk.get_imu()) <= 8 :
                #         self.imu_ok = True
                #一般轉頭避障
                elif (self.image.deep_x < 17 and self.image.deep_x >= 14) or (self.image.deep_x <= -14 and self.image.deep_x > -17) :
                    if  abs(self.walk.get_imu()) > 5 :  #(abs(self.walk.get_imu()) > 5) and (not self.imu_ok)
                        while  abs(self.walk.get_imu()) > 5 :
                            self.walk.move('imu_fix')
                    # elif abs(self.walk.get_imu()) < 5:
                        # if (self.image.b_y_max >= 200) and (self.need_imu_back) and (self.image.center_deep != 24) :        #離障礙物太近-->後退
                        #     while self.image.b_y_max >= 200 :
                        #         self.image.calculate()
                        #         self.walk.move('small_back') 
                        #     self.need_imu_back = False
                    self.image.calculate()
                    if (( self.image.left_deep < 14 ) and ( self.image.right_deep < 14 ) and ( self.image.center_deep < 14 )) or (self.image.line_at_left and self.image.center_deep < 14 and self.image.right_deep < 14) or (self.image.line_at_right and self.image.center_deep < 14 and self.image.left_deep < 14):
                        self.Turn_Head()
                        self.imu_ok = True

                    elif  self.image.deep_sum_l >= self.image.deep_sum_r : 
                        while abs(self.walk.get_imu()) < 40:    
                            self.walk.move('turn_left')
                        self.imu_ok = True
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
                            
                    elif  self.image.deep_sum_l < self.image.deep_sum_r :
                        while abs(self.walk.get_imu()) < 40:    
                            self.walk.move('turn_right')
                        self.imu_ok = True
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
                elif (6 >= self.image.deep_x >= -6) or (abs(self.image.deep_x) >= 17) :                  #最高速直走
                    self.walk.move('max_speed')
                    self.red_avoid_distance = True
                    if self.image.line_at_left or self.image.line_at_right:
                        self.imu_ok = True
                    elif self.image.deep_center_y < 6 and  3 >= self.image.deep_x >= -3:
                        self.imu_ok = False
                    # elif 3 >= self.image.deep_x >= -3 :
                    #     self.imu_ok = False
                    else : 
                        pass
                else :
                    pass
            # #沒看到障礙物
            elif self.image.deep_y == 24:
            # elif self.image.deep_center_y == 24:
                self.walk.move('max_speed')
            
                    
        if not send.is_start :
            send.sendSensorReset(1,1,1)
            if self.start_walking :
                send.sendContinuousValue(0,0,0,0,0)
                send.sendBodyAuto(0,0,0,0,1,0)
                time.sleep(0.5)
                # send.sendBodySector(29)
                # time.sleep(1.5)
                self.start_walking = False
            # send.sendContinuousValue(0,0,0,0,0)
            # self.walk.move('stay')

if __name__ == '__main__':

    try:
        aaaa = rospy.init_node('talker', anonymous=True)
        walk = Walk()
        send.use_new_color_mask = False
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
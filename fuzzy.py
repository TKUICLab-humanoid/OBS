#!/usr/bin/env python
#coding=utf-8
# from turtle import st
import rospy
import numpy as np
import sys
sys.path.append('/home/iclab/Desktop/adult_hurocup/src/strategy')
from Python_API import Sendmessage
# from hello1 import Sendmessage
from image import deep_calculate
# from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tku_msgs.msg import camera
import cv2 
import sys
import time
import math
import skfuzzy as fuzz
from skfuzzy import control as ctrl



#   很多判斷後的動作基本都一樣,可能要討論要怎再整合 可能也會縮短行數
#   image 參數輸出
#   ddd 參數整理


deep            = deep_calculate()
send            = Sendmessage()
CRMAX           = 65 #R_Max       65 55
CRMIN           = 55 #R_MIn       55 45
HEAD_HEIGHT     = 1550
FOCUS_MATRIX    = [7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7]
SMALL_FORWARD_X         = 1500                                                     
SMALL_FORWARD_Y         =  200                                                     
SMALL_FORWARD_THETA     =    1                                                       
TURN_RIGHT_X            = -500                                                     
TURN_RIGHT_Y            = 1200                                                     
TURN_RIGHT_THETA        =   -7                                                     
TURN_LEFT_X             =    0                                                     
TURN_LEFT_Y             =-1000                                                     
TURN_LEFT_THETA         =    7                                                     

class Walk():
    def __init__(self):
        self.image = Normal_Obs_Parameter()
        
    def move(self, action_id, z=0, sensor= 0):
        self.image.calculate()
        imu_flag = self.get_imu() < 0     #判斷是否<0
        slope_x_fix             = 100 if self.image.red_y_max < 150 else -100 if self.image.red_y_max > 200 else 0            #if y < 150則x=100,elif y > 200則x = -100,else x = 0         
        right_straight_y        = -700 if self.image.left_deep <= 11  else 600 if self.image.left_deep >= 13 else 0                  
        left_straight_y         = -700 if self.image.right_deep <= 11  else 600 if self.image.right_deep >= 13 else 0                  
        straight_90degree_fix   = 2 if ((imu_flag and abs(self.get_imu()) < 87) or (not imu_flag and abs(self.get_imu()) < 87)) else -2   #如果(flag=true且imu<87)則=2,或(flag=false且imu>-87)就=-2          
        turn_x                  =   self.straight_speed()*2 if self.image.yellow_center_deep < 12 else self.straight_speed()  
        turn_direction_x        =   TURN_RIGHT_X if self.get_imu() > 0 else TURN_LEFT_X  # fix_angle for turn_x
        actions             = { 'stay'                  : {'x':  100,              'y':   300,             'theta': 0                },
                                'max_speed'             : {'x':  2500,              'y':   200,             'theta': 1                },
                                'small_back'            : {'x': -1500,              'y':   400,             'theta':  0                },
                                'small_forward'         : {'x': SMALL_FORWARD_X,    'y':  SMALL_FORWARD_Y,  'theta': SMALL_FORWARD_THETA },
                                'turn'                  : {'x': turn_x,             'y':   300,             'theta': self.turn_angle() },
                                'imu_fix'               : {'x': turn_direction_x,   'y':   300,             'theta': self.imu_angle()  },
                                'slope_fix'             : {'x': turn_direction_x,   'y':   300,             'theta': self.slope()      },
                                'slope_right_translate' : {'x': 600 + slope_x_fix,  'y': -1200,             'theta': self.slope()      },
                                'slope_left_translate'  : {'x': slope_x_fix,        'y':  1800,             'theta': self.slope()      },
                                'turn_right'            : {'x': TURN_RIGHT_X,       'y':  TURN_RIGHT_Y,     'theta': TURN_RIGHT_THETA  },
                                'turn_right_back'       : {'x': TURN_LEFT_X,        'y':  TURN_LEFT_Y,      'theta': TURN_LEFT_THETA   },#.
                                'turn_left'             : {'x': TURN_LEFT_X,        'y':  TURN_LEFT_Y,      'theta': TURN_LEFT_THETA   },
                                'turn_left_back'        : {'x': TURN_RIGHT_X,       'y':  TURN_RIGHT_Y,     'theta': TURN_RIGHT_THETA  },#.
                                'face_right_forward'    : {'x': SMALL_FORWARD_X,    'y':  SMALL_FORWARD_Y + right_straight_y ,    'theta': SMALL_FORWARD_THETA + straight_90degree_fix    },
                                # 'right_right'         : {'x': SMALL_FORWARD_X,    'y':  SMALL_FORWARD_Y + straight_y_fix,     'theta': SMALL_FORWARD_THETA + straight_90degree_fix    },
                                'face_left_forward'     : {'x': SMALL_FORWARD_X,    'y':  SMALL_FORWARD_Y + left_straight_y,     'theta': SMALL_FORWARD_THETA + straight_90degree_fix   },
                                # 'left_left'           : {'x': SMALL_FORWARD_X,    'y':  SMALL_FORWARD_Y + straight_y_fix,     'theta': SMALL_FORWARD_THETA + straight_90degree_fix    },
                                'preturn_left'          : {'x': TURN_LEFT_X,        'y':  TURN_LEFT_Y,      'theta': TURN_LEFT_THETA   },
                                'preturn_right'         : {'x': TURN_RIGHT_X,       'y':  TURN_RIGHT_Y,     'theta': TURN_RIGHT_THETA  }}
        action              = actions.get(action_id,None)   #actions.get(actions_id) or None 字典沒值就None
        if action is not None:
            x              = action['x']
            y              = action['y']
            theta          = action['theta']
            send.sendContinuousValue(x, y, z, theta, sensor)
        print(action_id)

    def imu_yaw_ini(self):
        self.imu_yaw = 0

    def get_imu(self):
        self.imu_yaw = send.imu_value_Yaw
        return self.imu_yaw 

    def turn_angle(self):
        self.image.calculate()
        turn_ranges = [ (17, -6), 
                        (12, -6), 
                        (8,  -5), 
                        (6,  -4), 
                        (4,  -3), 
                        (2,  -2),  
                        (0,   0),
                        (-2,  2),
                        (-4,  3),
                        (-6,  4),
                        (-8,  5),
                        (-12, 6),
                        (-17, 6)]
        for turn_range in turn_ranges:           #將turn_ranges寫入turn_range
            if  self.image.deep_x >= turn_range[0]:
                return turn_range[1]
        return 0                                 #都不在範圍則給0
    
    def imu_angle(self):      
        imu_ranges = [  (90,  -5), 
                        (60,  -4), 
                        (45,  -4), 
                        (20,  -3), 
                        (10,  -2), 
                        (5,   -1), 
                        (2,   -1), 
                        (0,    0),
                        (-2,    3),
                        (-5,    3),
                        (-10,   3),
                        (-20,   4),
                        (-45,   4),
                        (-60,   5),
                        (-90,   6)]
        for imu_range in imu_ranges:           #將imu_range寫入imu_ranges
            if self.imu_yaw >= imu_range[0]:
                return imu_range[1]
        return 0

    def slope(self):
#-------------------fix to l---------------------
        if deep.slope > 0:          
            slopel_ranges = [(2,     0), 
                             (1,     5), 
                             (0.3,   5), 
                             (0.2,   4), 
                             (0.15,  4), 
                             (0.1,   4), 
                             (0.06,  3), 
                             (0.03,  3), 
                             (0,     0)]
            for slopel_range in slopel_ranges:
                if deep.slope >= slopel_range[0]:
                    return slopel_range[1]
            return 0
#--------------------fix to r--------------------
        elif deep.slope <= 0:     
            slopel_ranges = [(-2,      0), 
                             (-1,     -5), 
                             (-0.3,   -5), 
                             (-0.2,   -4), 
                             (-0.15,  -4), 
                             (-0.1,   -4), 
                             (-0.06,  -3), 
                             (-0.03,  -3), 
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
        speed_ranges = [(24,    2500), 
                        (20,    2200), 
                        (16,    1800), 
                        (14,    1500), 
                        (12,    1200), 
                        (8,      900), 
                        (6,      600), 
                        (3,      300), 
                        (0,        0)]
        for speed_range in speed_ranges:
            if self.image.deep_y >= speed_range[0]:
                return speed_range[1]
        return 0

class Fuzzy():
    def __init__(self):
        self.image = Normal_Obs_Parameter()
        # self.deep = deep_calculate()

    def Fuzzy(self):
        self.image.calculate()
        # self.deep.blue_obs()
        Dx   = ctrl.Antecedent(np.arange(-16, 17, 1), 'Dx')
        Turn = ctrl.Consequent(np.arange(-13, 14, 1), 'Turn')

        Dx['R_Full']    = fuzz.trimf(Dx.universe,   [-16, -16, - 8])
        Dx['R_O']       = fuzz.trimf(Dx.universe,   [-14, -12, - 2])
        Dx['No']        = fuzz.trimf(Dx.universe,   [- 3,   0,   3])
        Dx['L_O']       = fuzz.trimf(Dx.universe,   [  2,  12,  14])
        Dx['L_Full']    = fuzz.trimf(Dx.universe,   [  8,  16,  16])

        Turn['T_Right'] = fuzz.trimf(Turn.universe, [-13, -13, - 7])
        Turn['T_R']     = fuzz.trimf(Turn.universe, [- 8, - 8, - 2])
        Turn['NoTurn']  = fuzz.trimf(Turn.universe, [- 3,   0,   3])
        Turn['T_L']     = fuzz.trimf(Turn.universe, [  2,   8,   8])
        Turn['T_Left']  = fuzz.trimf(Turn.universe, [  7,  13,  13])

        rule1 = ctrl.Rule(Dx['L_Full'], Turn['T_Right'])
        rule2 = ctrl.Rule(Dx['L_O'],    Turn['T_R'])
        rule3 = ctrl.Rule(Dx['No'],     Turn['NoTurn'])
        rule4 = ctrl.Rule(Dx['R_O'],    Turn['T_L'])
        rule5 = ctrl.Rule(Dx['R_Full'], Turn['T_Left'])

        # 建立模糊控制系統
        turn_ctrl   = ctrl.ControlSystem([rule1, rule2, rule3,rule4,rule5])
        turning     = ctrl.ControlSystemSimulation(turn_ctrl)
        # 執行模糊控制
        Dx_value    = self.image.deep_x  
        turning.input['Dx'] = Dx_value
        turning.compute()
        # 解模糊得到控制輸出
        self.Turn_value = int(turning.output['Turn'])
        # rospy.loginfo(f'Dx =  {self.image.deep_x}')
        # rospy.loginfo(f'Turn =  {self.Turn_value}') 
        # print("Dx = ",self.image.deep_x)
        # print("Turn = ",self.Turn_value)

        Dy      = ctrl.Antecedent(np.arange(0, 25, 1), 'Dy')
        Speed   = ctrl.Consequent(np.arange(0, 26, 1), 'Speed')
        Dy['C_OBS'  ]   = fuzz.trimf(Dy.universe,    [ 0,  0,  7])
        Dy['NC_OBS' ]   = fuzz.trimf(Dy.universe,    [ 2, 12, 12])
        Dy['NOR_OBS']   = fuzz.trimf(Dy.universe,    [ 6, 17, 17])
        Dy['NF_OBS' ]   = fuzz.trimf(Dy.universe,    [10, 19, 19])
        Dy['F_OBS'  ]   = fuzz.trimf(Dy.universe,    [18, 24, 24])
        Speed['SLOW' ]  = fuzz.trimf(Speed.universe, [ 0,  0,  0])
        Speed['N_S'  ]  = fuzz.trimf(Speed.universe, [ 0,  5,  6])
        Speed['NOR_S']  = fuzz.trimf(Speed.universe, [ 5, 15, 20])
        Speed['N_F'  ]  = fuzz.trimf(Speed.universe, [13, 22, 25])
        Speed['FAST' ]  = fuzz.trimf(Speed.universe, [20, 27, 27])
        
        rule6  = ctrl.Rule(Dy['C_OBS'   ],  Speed['SLOW'    ])
        rule7  = ctrl.Rule(Dy['NC_OBS'  ],  Speed['N_S'     ])
        rule8  = ctrl.Rule(Dy['NOR_OBS' ],  Speed['NOR_S'   ])
        rule9  = ctrl.Rule(Dy['NF_OBS'  ],  Speed['N_F'     ])
        rule10 = ctrl.Rule(Dy['F_OBS'   ],  Speed['FAST'    ])
        # 建立模糊控制系統
        speed_ctrl = ctrl.ControlSystem([rule6, rule7, rule8,rule9,rule10])
        speeding = ctrl.ControlSystemSimulation(speed_ctrl)
        # 執行模糊控制
        Dy_value =  self.image.deep_center_y
        speeding.input['Dy'] = Dy_value
        speeding.compute()
        # 解模糊得到控制輸出
        self.Speed_value = 100*(int(speeding.output['Speed']))
        rospy.loginfo(f'dy =  {self.image.deep_center_y}') 
        rospy.loginfo(f'speed =  {self.Speed_value}')  

class Normal_Obs_Parameter:
    def __init__(self):
        self.deep                       = deep_calculate()
        self.line_at_left               = False
        self.line_at_right              = False
        self.at_reddoor_flag            = False
        self.deep_y                     = 24
        self.deep_x                     = 0
        self.yellow_center_deep         = 0       #黃色中心深度值
        self.y_move                     = 0
        self.deep_sum                   = 0
        self.left_deep_sum              = 0
        self.right_deep_sum             = 0
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

    def reddoor_info(self):
        self.red_y_max = send.color_mask_subject_YMax[5][0]
        self.red_x_min = send.color_mask_subject_XMin[5][0] 
        self.red_x_max = send.color_mask_subject_XMax[5][0]
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

    def yellow_obs_info(self):
        self.y_deep_y           = min(deep.yellow_deep)
        self.y_deep_sum         = sum(deep.yellow_deep)
        self.y_left_deep        = deep.yellow_deep[2]
        self.y_right_deep       = deep.yellow_deep[30]
        self.yellow_center_deep = deep.yellow_deep[16]
        self.y_deep_left_sum    = sum(deep.yellow_deep[0:15])
        self.y_deep_right_sum   = sum(deep.yellow_deep[16:31])
    
    def blue_obs_info(self):
        # deep.blue_obs()
        # print(send.color_mask_subject_YMax[2])
        if send.color_mask_subject_cnts[2] > 0:
            # self.b_y_max        = send.color_mask_subject_YMax[2]
            # aa = self.b_y_max[0]
            # blue_Xmax_array = np.array(send.color_mask_subject_XMax[2])
            print(np.array(send.color_mask_subject_XMax[2])[2])
            # aa = self.b_y_max[0])
            # self.b_deep_y       = min(deep.blue_deep)
            # self.b_deep_sum     = sum(deep.blue_deep)
            # self.b_left_deep    = deep.blue_deep[2]
            # self.b_right_deep   = deep.blue_deep[30]
            # self.b_center_deep  = deep.blue_deep[16]
            # print("cnt",send.color_mask_subject_cnts[2])
            # print("cnt  = ",send.color_mask_subject_cnts[2])
            # if send.color_mask_subject_cnts[2] == 2:
            #     self.blue_rightside     = max(send.color_mask_subject_XMin[2])
            #     self.blue_leftside      = min(send.color_mask_subject_XMax[2])
            # else:
            #     self.blue_rightside = 0
            #     self.blue_leftside = 0
            # print("xMax = ",send.color_mask_subject_XMax[2])
            
            # print("xMin = ",send.color_mask_subject_XMin[2])
            # print("max  = ",self.blue_leftside)
            # print("min  = ",self.blue_rightside)
    def calculate(self):
        if send.color_mask_subject_size[5][0] > 5000:                      #有紅時計算紅門資訊
            self.reddoor_info()
        if send.color_mask_subject_cnts[2] > 0:
            self.blue_obs_info()
        deep.obs()
        self.at_reddoor_flag = False
#----------------Filter_matrix-------------------
        filter_matrix          = [max(0, a - b) for a, b in zip(FOCUS_MATRIX, deep.obs_deep)] #將FOCUS_MATRIX和deep.blue_yellow_deep結合後相減，如果<0則給0，>0則給相減值
        x_center_num           = sum(i for i, num in enumerate(FOCUS_MATRIX - np.array(deep.obs_deep)) if num >= 0)#np.array=>[1,2]=>[1 2]，list(enumerate([1 2],[3 5]))=[(0,-2),(1,-3)] ##計算差值>=0的index(i)總和
        x_center_cnt           = np.sum(np.array(FOCUS_MATRIX) - np.array(deep.obs_deep) >= 0) #有幾個相減後>0
        x_center               = (x_center_num / x_center_cnt) if x_center_cnt > 0 else 0
        left_weight_matrix     = list(range(32))            #0~31
        right_weight_matrix    = list(range(31,-1,-1))      #31~0
        right_weight           = np.dot(filter_matrix,  right_weight_matrix)#內積
        left_weight            = np.dot(filter_matrix,  left_weight_matrix)
        self.deep_y            = min(deep.obs_deep)
        self.deep_sum          = sum(deep.obs_deep)
        self.deep_center_y     = min(deep.obs_deep[10:23])
        self.left_deep         = deep.obs_deep[4]
        self.right_deep        = deep.obs_deep[28]
        self.center_deep       = deep.obs_deep[16]
        x_boundary             = 31 if left_weight > right_weight else 0

        if send.color_mask_subject_cnts[1] == 2 and send.color_mask_subject_YMax[0][1] > 150 and send.color_mask_subject_YMax[1][1] > 150:
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

class Obs:
    def __init__(self):
        self.image          = Normal_Obs_Parameter()
        self.walk           = Walk()
        self.fuzzy          = Fuzzy()
        self.blue_at_left           = False
        self.blue_at_right          = False
        self.redoor_distence        = False
        self.need_fix_slope         = False
        self.first_reddoor          = False
        self.start_walking          = False
        self.imu_ok                 = False
        self.need_imu_back          = True

    def red_door(self):
        if not self.first_reddoor:
            self.first_reddoor = True
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT,100)
            time.sleep(0.2)
            send.sendContinuousValue(0, 0 , 0 , 0 , 0)
        
        elif self.first_reddoor :
            while self.image.red_y_max > 220 :
                self.image.calculate()
                self.walk.move('small_back')

            if abs(deep.slope) > 0.03 and self.need_fix_slope:                     #不平行紅門時修斜率
                while abs(deep.slope) > 0.03 :
                    self.walk.move('slope_fix')
                self.need_fix_slope = False
            else :
                if (self.image.red_y_max < 210) and not self.redoor_distence :     #前後距離修正（值越大離門越近） 55/65   離紅門太遠時前進
                    self.walk.move('small_forward')
                elif (self.image.red_y_max > 220) and not self.redoor_distence :   #前後距離修正（值越大離門越近） 55/65   離紅門太近時候退
                    self.walk.move('small_back')
                else :                          #判斷須左移或右移 都不需則進爬
                    self.redoor_distence = True
                    if self.image.red_x_min < 2 and self.image.red_x_max > 315 :
                        if (self.image.red_x_max == 0 and self.image.red_x_min == 0 and self.image.blue_leftside <= 28 and self.blue_at_right ) or (self.image.b_x_max == 0 and self.image.b_x_min == 0 and self.image.blue_rightside > 292 and self.blue_at_left ) or (self.image.b_x_max == 0 and self.image.b_x_min == 0 and self.image.blue_rightside == 0 and self.image.blue_leftside == 0):
                            while abs(deep.slope) > 0.03 or (self.need_fix_slope) :
                                self.walk.move('slope_fix')
                            self.crawl()
                        elif (self.image.b_x_min < 2 and self.image.b_x_max > 20):
                            self.blue_at_right = True
                            self.blue_at_left = False
                            self.walk.move('slope_right_translate')
                        elif (self.image.b_x_max > 315 and self.image.b_x_min < 305):
                            self.blue_at_left = True
                            self.blue_at_right = False
                            self.walk.move('slope_left_translate')
                        else :
                            if self.blue_at_right :
                                self.walk.move('slope_right_translate')
                            elif self.blue_at_left :
                                self.walk.move('slope_left_translate')

                    elif self.image.red_x_min < 2 and self.image.red_x_max < 315 : 
                        self.walk.move('slope_left_translate')

                    elif self.image.red_x_min > 2 and self.image.red_x_max > 315 : 
                        self.walk.move('slope_right_translate') 

    def crawl(self):
        while self.image.red_y_max < CRMIN or self.image.red_y_max > CRMAX or CRMIN <= self.image.red_y_max <= CRMAX:
            if(self.image.red_y_max < CRMIN):            #前進修正
                self.walk.slope()
                self.walk.move('small_forward')
            elif(self.image.red_y_max > CRMAX):          #後退修正
                self.walk.slope()
                self.walk.move('small_back')
            elif CRMIN <= self.image.red_y_max <= CRMAX:    #爬                                #不平行紅門時修斜率
                while abs(deep.slope) > 0.03:
                    self.walk.slope()
                    self.walk.move('slope_fix')
                send.sendContinuousValue(0, 0 , 0 , 0 , 0) 
                time.sleep(1)
                send.sendBodyAuto(0,0,0,0,1,0)
                time.sleep(2)
                #send.sendBodySector(299)    #基礎站姿29！！！！！！！！！！！！！！！！！！
                send.sendBodySector(6666)
                time.sleep(2.2)
                # send.sendBodySector(123)
                send.sendBodySector(7777)
                time.sleep(6)
                while crawl_cnt < 3:                #避免門下建模有問題 固定爬三次才抬頭
                    send.sendBodySector(456)
                    time.sleep(2.8)
                    # time.sleep(0.3)
                    crawl_cnt += 1
                send.color_mask_subject_YMax[1][0] = 0
                self.b_y_max = 0
                send.sendHeadMotor(1,2048,100)
                send.sendHeadMotor(2,2500,100)
                time.sleep(1)
                while crawl_cnt < 8:                   #邊爬邊判斷是否離障礙物太近
                    #Image_Init()
                    strategy.Normal_Obs_Parameter()
                    time.sleep(0.1)
                    if (self.b_y_max >= 85 and send.color_mask_subject_size[2][0] > 5000) or (send.color_mask_subject_YMax[1][0] >= 60 and send.color_mask_subject_size[1][0] > 5000):
                        break
                    else:
                        send.sendBodySector(456)
                        time.sleep(2.8)
                        #time.sleep(0.1)
                        crawl_cnt += 1
                if crawl_cnt > 6 :
                    send.sendBodySector(8888)
                    time.sleep(12.5)
                    send.sendBodySector(29)    #基礎站姿29！！！！！！！！！！！！！！！！！！
                    time.sleep(0.5)
                    send.sendHeadMotor(1,2048,100)
                    send.sendHeadMotor(2,HEAD_HEIGHT,100)
                    #send.sendBodySector(15)#小白
                    #send.sendBodySector(16)#小黑
                    time.sleep(0.5)
                    send.sendBodySector(5555)
                    time.sleep(2.5)
                    send.sendBodyAuto(0,0,0,0,1,0)
                    break 
                else :
                    send.sendBodySector(8888)
                    time.sleep(12.5)
                    send.sendBodySector(29)    #基礎站姿29！！！！！！！！！！！！！！！！！！
                    time.sleep(0.5)
                    send.sendHeadMotor(1,2048,100)
                    send.sendHeadMotor(2,HEAD_HEIGHT,100)
                    #send.sendBodySector(15)#小白
                    #send.sendBodySector(16)#小黑
                    time.sleep(1.5)
                    send.sendBodyAuto(0,0,0,0,1,0)
                # send.sendBodySector(1113)
                # time.sleep(14.4)
                # time.sleep(1.5)
                # send.sendBodySector(299)    #基礎站姿29！！！！！！！！！！！！！！！！！！
                # time.sleep(1.5)
                # send.sendHeadMotor(1,2048,100)
                # send.sendHeadMotor(2,1550,100)
                # time.sleep(0.5)
                # send.sendBodyAuto(0,0,0,0,1,0)
                # time.sleep(1)
                # break

    def turn_head(self):
        self.walk.move('stay')
        if not self.image.line_at_right and not self.image.line_at_left: 
            time.sleep(1)
            send.sendHeadMotor(1,1497,100)
            send.sendHeadMotor(2,HEAD_HEIGHT,100)
            send.sendHeadMotor(1,1497,100)
            send.sendHeadMotor(2,HEAD_HEIGHT,100)
            send.sendHeadMotor(1,1497,100)
            send.sendHeadMotor(2,HEAD_HEIGHT,100)
            time.sleep(1.8) 
            self.image.calculate()
            self.image.right_deep_sum = self.image.deep_sum
            send.sendHeadMotor(1,2599,100)
            send.sendHeadMotor(2,HEAD_HEIGHT,100)
            send.sendHeadMotor(1,2599,100)
            send.sendHeadMotor(2,HEAD_HEIGHT,100)
            send.sendHeadMotor(1,2599,100)
            send.sendHeadMotor(2,HEAD_HEIGHT,100)
            time.sleep(1.8)
            self.image.calculate()
            self.left_deep_sum = self.image.deep_sum
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT+100,100)
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT+100,100)
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT+100,100)
            time.sleep(0.3)
        else :
            self.image.right_deep_sum = 0
            self.image.left_deep_sum = 0
        if (self.image.right_deep_sum > self.image.left_deep_sum) or self.image.line_at_left:        #右轉
            self.image.calculate()
            if (self.image.b_center_deep > 7):                   #靠近障礙物
                while ( self.image.b_center_deep > 7 ):
                    self.image.calculate()
                    self.walk.move('small_forward')
            elif ( self.image.b_center_deep < 6 ):                #遠離障礙物
                while ( self.image.b_center_deep < 6 ):
                    self.image.calculate()
                    self.walk.move('small_back')
            if abs(self.walk.get_imu()) < 60:                   #靠近後右旋轉至90度
                while abs(self.walk.get_imu()) < 60:
                    self.walk.move('turn_right')
            send.sendHeadMotor(1,2647,100)
            send.sendHeadMotor(2,1550,100) 
            time.sleep(0.5)
            self.image.calculate()
            send.sendContinuousValue(0, 0 , 0 , 0 , 0)
            if abs(self.image.deep_x) >= 1 :               #直走且imu修正
                while abs(self.image.deep_x) >= 1 :     #轉頭後直走 平移修正
                    self.image.calculate()  
                    if abs(self.walk.get_imu()) > 87 :          #視步態更動
                        if (self.image.left_deep != 24) :
                            self.walk.move('face_right_forward')
                        else :
                            break
                    # if center_deep == 24 and right_deep == 24:# MRT
                    #     Move(Straight_status = 15)
                    #     time.sleep(0.5)
                    #     break
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT,100) 
            time.sleep(0.5)
            self.image.calculate()
            if abs(self.walk.get_imu()) > 60:             #右轉回正
                while abs(walk.get_imu()) > 60:
                    self.walk.move('turn_right_back')
        elif (self.image.left_deep_sum > self.image.right_deep_sum) or self.image.line_at_right:         #左轉
            self.image.calculate()
            if (self.image.b_center_deep > 3):                   #靠近障礙物
                while ( self.image.b_center_deep > 3 ):
                    self.image.calculate()
                    self.walk.move('small_forward') 
            elif ( self.image.b_center_deep < 2 ):                #遠離障礙物
                while ( self.image.b_center_deep < 2 ):
                    self.image.calculate()
                    self.walk.move('small_back') 
            if abs(self.walk.get_imu()) < 65:                           #靠近後轉至90度
                while abs(self.walk.get_imu()) < 65:
                    self.walk.move('turn_left')
            send.sendHeadMotor(1,1447,100)
            send.sendHeadMotor(2,1550,100) 
            time.sleep(0.5)
            self.image.calculate()
            send.sendContinuousValue(0, 0 , 0 , 0 , 0)
            if  abs(self.image.deep_x) >= 1:                          #直走且imu修正
                while  abs(self.image.deep_x) >= 1 :        #轉頭後直走 平移修正
                    self.image.calculate()
                    if abs(self.walk.get_imu()) > 85 :          #視步態更動
                        if (self.image.right_deep != 24) :
                            self.walk.move('face_left_forward')
                        else :
                            break
                    # if center_deep == 24 and left_deep == 24:#MRT
                    #     Move(Straight_status = 15)
                    #     time.sleep(0.5)
                    #     break
            send.sendHeadMotor(1,2048,100)
            send.sendHeadMotor(2,HEAD_HEIGHT,100) 
            time.sleep(0.5)
            self.image.calculate()
            #----看到黃線
            if abs(self.walk.get_imu()) > 50:               #左轉回正
                while abs(self.walk.get_imu()) > 50:    
                    self.walk.move('turn_left_back')

    def main(self):
        if send.is_start :
            self.image.calculate()
        #=============================strategy=============================
            if not self.start_walking :                        #指撥後初始動作
                #self.walk.imu_yaw_ini()
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
                while abs(self.walk.get_imu()) < 60:
                    self.walk.move('preturn_left')
                    rospy.loginfo(f'imu =  {self.walk.get_imu()}')
                self.preturn_left = False
            elif self.preturn_right:                        #指定初始向右旋轉 
                while abs(self.walk.get_imu()) < 55:
                    self.walk.move('preturn_right')
                    rospy.loginfo(f'imu =  {self.walk.get_imu()}')
                self.preturn_right = False
            if self.image.at_reddoor_flag:
                self.red_door()
            else :
                if self.image.deep_y < 24:
                    # self.image.calculate()
            #         if self.image.line_at_right :
            #             if self.image.y_deep_left_sum > self.image.y_deep_right_sum :
            #                 self.line_at_right = True
            #             elif (self.image.y_deep_left_sum < self.image.y_deep_right_sum) or (self.image.y_deep_right_sum > 350) :
            #                 self.image.line_at_right = False
            #         elif self.image.line_at_left :
            #             if self.image.y_deep_left_sum < self.image.y_deep_right_sum :
            #                 self.image.line_at_left = True
            #             elif (self.image.y_deep_left_sum > self.image.y_deep_right_sum) or (self.image.y_deep_right_sum > 350) :
            #                 self.image.line_at_left = False

                    if 13 > self.image.deep_x > 3 :        #turn right
                        self.walk.straight_speed()
                        if (self.image.b_y_max >= 170) and ( abs(self.walk.get_imu()) <= 5 ) and (self.need_imu_back)  and (abs(self.image.deep_x) > 3) and (self.image.center_deep != 24):        #離障礙物太近-->後退
                            while (self.image.b_y_max >= 170):
                                self.image.calculate()
                                self.walk.move('small_back') 
                                if self.image.b_y_max > 0 :
                                    break
                            self.need_imu_back = False
                        if ((abs(self.walk.get_imu()) > 5) and (not self.imu_ok)) and (self.image.deep_x >= 5) :       #IMU修正

                            self.walk.move('imu_fix')
                        else:
                            self.walk.move('turn')

                        if abs(self.walk.get_imu()) <= 5 :
                            self.imu_ok = True
                    elif -3 > self.image.deep_x > -13 :     #turn left
                        self.walk.straight_speed()
                        if (self.image.b_y_max >= 170) and ( abs(self.walk.get_imu()) <= 5 ) and (self.need_imu_back) and (abs(self.image.deep_x) > 3) and (self.image.center_deep != 24):        #離障礙物太近-->後退
                            while (self.image.b_y_max >= 170):
                                self.image.calculate()
                                self.walk.move('small_back') 
                                if self.image.b_y_max > 0 :
                                    break
                            self.need_imu_back = False
                        if ((abs(self.walk.get_imu()) > 5) and (not self.imu_ok)) and (self.image.deep_x <= -7) :      #IMU修正
                            self.walk.imu_angle()
                            self.walk.move('imu_fix')
                        else:
                            self.walk.move('turn')

                        if abs(self.walk.get_imu()) <= 5 :
                            self.imu_ok = True
                    elif (self.image.deep_x < 17 and self.image.deep_x >= 13) or (self.image.deep_x <= -13 and self.image.deep_x > -17) :
                        if (self.image.b_y_max >= 170) and ( abs(self.walk.get_imu()) <= 5 ) and (self.need_imu_back) and (abs(self.image.deep_x) > 3) and (self.image.center_deep != 24) :        #離障礙物太近-->後退
                            while (self.image.b_y_max >= 170):
                                self.image.calculate()
                                self.walk.move('small_back') 
                                if self.image.b_y_max > 0 :
                                    break
                            self.need_imu_back = False
                        if ( abs(self.walk.get_imu()) > 5) and (not self.imu_ok) :                   #IMU修正
                            while ( abs(self.walk.get_imu()) > 5) and (not self.imu_ok) :
                                self.walk.move('imu_fix')
                                self.image.calculate()
                                if abs(self.walk.get_imu()) < 5:        #轉頭策略
                                    if ( self.image.b_left_deep < 15 ) and ( self.image.b_right_deep < 15 ) and ( self.image.b_center_deep < 20 ):
                                        self.turn_head()
                                        self.imu_ok = True
                                        break
                                    else :
                                        break
                                else:
                                    pass
                        elif (abs(self.walk.get_imu()) < 5) and (self.imu_ok):
                            self.walk.move('imu_fix')
                            
                            if abs(self.walk.get_imu()) < 5: 
                                if ( self.image.b_left_deep < 13 ) and ( self.image.b_right_deep < 13 ) and ( self.image.b_center_deep < 18 ):
                                    self.turn_head()
                                else :
                                    self.walk.move('turn')
                            else:
                                pass

                    elif (3 >= self.image.deep_x >= -3) or (abs(self.image.deep_x) >= 17):                  #最高速直走
                        self.walk.move('max_speed')
                        if self.image.deep_x == 0 :
                            self.imu_ok = False
                            self.need_imu_back = True
                            
                elif self.image.deep_y == 24:
                    self.walk.move('max_speed')
                    
        if not send.is_start :
            # self.fuzzy.Fuzzy()
            self.image.blue_obs_info()
            # self.fuzzy.Turn_value
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

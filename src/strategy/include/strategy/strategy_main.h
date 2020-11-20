#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <stdio.h>
#include <std_msgs/String.h>
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "tku_libs/RosCommunication.h"
#include <ctime>
#include <iostream>
#include <string>
//////////////////////////////////////////////////////////

#include <stdlib.h>
#include "strategy/DeepMatrix.h"
#include <sys/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>


#define DeepMatrixSize 32
#define pi 3.14159

ros::Subscriber DeepMatrix_subscribe;

enum strategy_state
{
    P_INIT,
    P_MATRIX_CALCULATE,
    P_FIND_WALKINGSTATE,
    P_FM_TURNHEAD,
    P_WALKINGGAIT,
    P_DOOR,
    P_CRAWL
};

struct obstacle_data
{
    int x;
    int y;
    int x_min;
    int x_max;
    int y_min;
    int y_max;
    int size;
};

enum walking_command
{
    FORWARD_FAST,
    FORWARD_NORMAL,
    FORWARD_SLOW,
    RMOVE_FAST,
    RMOVE_NORMAL,
    RMOVE_SLOW,
    LMOVE_FAST,
    LMOVE_NORMAL,
    LMOVE_SLOW,
	DIRmap_RIGHT,
	DIRmap_LEFT,
    WC_BIGLEFT,
    WC_MIDLEFT,
    WC_LEFT,
    WC_BIGRIGHT,
    WC_MIDRIGHT,
    WC_RIGHT,
    continuousValue_Ry,
    continuousValue_Ly,
	continuousValue_Rt,////add
	continuousValue_Lt,////add
	continuousValue_R2t,////add
	continuousValue_L2t,////add
	RMOVE_DOOR,
	LMOVE_DOOR
};

struct label_model_coordinates
{
        float x, y;
};


enum direction
{
    Right,
    Left
};

enum Head_direction
{
    RHD_Center,
    RHD_Right,
    RHD_Left
};

using namespace std;

class KidsizeStrategy 
{
public:
	KidsizeStrategy(ros::NodeHandle &nh) 
	{
		DeepMatrix_subscribe = nh.subscribe("/strategy/DeepMatrix_Topic", 1, &KidsizeStrategy::GetDeepMatrix,this);
		strategy_info = StrategyInfoInstance::getInstance();
		tool = ToolInstance::getInstance();
		ros_com = RosCommunicationInstance::getInstance();
	};
	~KidsizeStrategy(){};
	void strategymain();

	RosCommunicationInstance *ros_com;
	ToolInstance *tool;
	StrategyInfoInstance *strategy_info;

	strategy_state m_state;
	walking_command spec_RLmove_state;
	walking_command walking_state;
	walking_command pre_walking_state;
	obstacle_data obs_data;
	direction turn_direction;
	Head_direction head_direction;
	Head_direction door_direction;
	label_model_coordinates red_obs_left_coordinates;
	label_model_coordinates red_obs_right_coordinates;

	vector<obstacle_data> m_obs_vector;
	vector<obstacle_data> m_finish_obs_vector;

	ObjectData blue_obs;
	ObjectData blue_obs_second;
	ObjectData blue_obs_third;
	ObjectData P_DOOR_blue;
	ObjectData P_DOOR_blues;
	SensorMode IMU_continuous;
	SensorMode IMU_single;

	int DeepMatrixValue[32];
	int FilterMatrix[32];
	int RMoveValue = 0;
	int LMoveValue = 0;
	int true_RMoveValue=0;
	int true_LMoveValue=0;
	int compareObs;
	int compareObssize;
	int bigobs;
	int decideforward;
	int decidemove;
	int LRmove_cnt;
	int fixed_LRmove_cnt;
	int continuousValue_x;
	int continuous_x_offset;
	int continuous_y_offset;
	int continuous_y_offset_RIGHT;
	int continuous_y_offset_LEFT;
	int continuous_theta_offset;
	int continuous_theta_offset_RIGHT;
	int continuous_theta_offset_LEFT;
	int continuous_x_offset_RIGHT;
	int continuous_x_offset_LEFT;
	int RHead_X;
	int LHead_X;
	int insideFMcnt;
	int first_cnt=0;
	int Rrightimage=0;
	int Rleftimage=0;
	int Ry_fastest;
	int Ly_fastest;
   	int dirmap[3];
	int dirdata[47];
	int cntTopYellow_x=0;
	int cntBottomYellow_x=0;
	int BottomYellowPoint=0;
	int MidYellowPoint=0;
	int TopYellowPoint=0;
        
	float slope_avg;         //red door slope
	float slope_avg_blue;    //blue obs slope
	float slope;
	float IMU_slope;
	float sidelineslope;

	bool in_reddoor_flag=false;	//add
    bool slope_flag=false;
	bool Blue_obs_flag = false;
	bool Red_Door_flag = false;
	bool first_enter_door = true;
	bool turnslope_flag = false;
	bool zero_flag = true;
	bool check_obs = false;
	bool checking_obs;
	bool check_road = false;
	bool face_to_door = false;
	bool Center_door = false;
	bool rest_flag = false;
	bool pcrawl_flag = false;
	bool stand_flag = false;
	bool red_modle_flag = false;
	bool check_LRmove_flag = false;
	bool special_obs_flag = false;
	bool Continuous_flag = false;
	bool first_continuous_flag = false;
	bool Turnhead_flag = false;
	bool turnhead_open_cnt = true;
	bool second_flag = false;
	bool check_hole = false;
	bool door_localization = false;
	bool first_move_flag =false;
	bool first_act_flag = true;
	bool p_door = false;
	bool movecnt = true;
	bool First_flag = false;  //第一個藍色障礙物旗標（判斷藍色障礙物在紅門下個數）
	bool crw_up = false;      //爬的過程中看紅門後的障礙物距離多少,決定要不要爬起
	bool hole_flag = true;
	bool sidelinewarning=false;
	bool rightsidelinewarning=false;
	bool leftsidelinewarning=false;
	bool sideline_zero_flag=false;
	bool twentyflag=false;
    int First_width = 0;
	int angle_offset = 0;
	int continous_angle_offest = 0;
	string parameter_path = "N";
	string m_state_string="a";
	string walking_state_string="a";
	void GetDeepMatrix(const strategy::DeepMatrix &msg);
	void SlopeCalculate();
	void facetodoorfun();
	void FaceToObsFun();	//when face to blue obs
	void initparameterpath();
	void load_dirtxt();
	void readwalkinggait();
    void turnslope();
	void give_angle();
	void IMUSlope();
	void FaceToFinialLineFun();
	void traverse();
	void sideline();
	void printinfo();
};

//bool isStart = false;


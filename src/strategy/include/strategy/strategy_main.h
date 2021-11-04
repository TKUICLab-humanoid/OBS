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
#include "strategy/GetParameter.h"
#include <sys/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>


#define DeepMatrixSize 32
#define pi 3.14159

ros::Subscriber DeepMatrix_subscribe;

using namespace std;

class KidsizeStrategy 
{
public:
	KidsizeStrategy(ros::NodeHandle &nh) 
	{
		DeepMatrix_subscribe = nh.subscribe("/strategy/GetParameter_Topic", 1, &KidsizeStrategy::GetParameter,this);
		strategy_info = StrategyInfoInstance::getInstance();
		tool = ToolInstance::getInstance();
		ros_com = RosCommunicationInstance::getInstance();
	};
	~KidsizeStrategy(){};
	void strategymain();

	RosCommunicationInstance *ros_com;
	ToolInstance *tool;
	StrategyInfoInstance *strategy_info;

	//////////////////////////////////////new strategy parameter ////////////////////////////////////////////

	
	/*********************stategy_main parameter ********************/
	
	enum strategy_state
	{
		INIT,
		AVOID,
		TURNHEAD,
		REDDOOR,
		CRAWL
	};

	/***************subscribe deepmatrix parameter******************/
	//0905++++
	int Dy = 0;
	int RD = 0;
	int LD = 0;
	int WR = 0;
	int WL = 0;
	int LeftblueOBS_XMax = 0;
	int RightblueOBS_XMin = 0;
	int L_XMAX = 0;
	int R_XMIN = 0;
	float slope_avg;
	float Dx = 0;
	bool in_reddoor_flag = false;
	bool b_obs_flag = false;
	bool y_obs_flag = false;
	int l_center_Dy = 0;
	int r_center_Dy = 0;
	bool one_b_flag = false;
	bool two_b_flag = false;
	int check_no_obs_cnt = 0;
	int layer_sum = 0;
	bool layer_flag = false;
	bool LeftHead_flag = false;
    bool RightHead_flag = false;
	int turn_WR = 0;
	int turn_WL = 0;
	bool reddoor_slope_ok_flag = false;
	bool imu_ok_flag = false;
	

	string parameter_path = "N";
	//0905++++
	/******************subscribe deepmatrix parameter******************/

	int continuous_angle_offset = 0;
	int continuous_speed = 0;
	bool init_flag = true;
	float IMU_Value = 0;
	float IMU_getValue = 0;
	
	/*********************stategy_main parameter ********************/
	strategy_state strategy_state;
	//walking_command walking_state;
	SensorMode IMU_continuous;
	int angle_offest = 0;
	bool Continuous_flag = false;

	/******************ini  parameter******************/

	struct walking_gait
	{
		int x;
		int y;
		int theta;
	};

	walking_gait stay;
	walking_gait Rmove;
	walking_gait Lmove;

	//0905++++
	int LeftMove_X;
	int LeftMove_Y;
	int LeftMove_T;
	int RightMove_X;
	int RightMove_Y;
	int RightMove_T;
	int LeftSlope_X;
	int LeftSlope_Y;
	int LeftSlope_T;
	int RightSlope_X;
	int RightSlope_Y;
	int RightSlope_T;
	//0905++++

	int maxspeed = 0;
	int midspeed = 0;
	int minspeed = 0;
	
	int b_dangerous_distance = 0;
	int y_dangerous_distance = 0;
	int continuousValue_x = 0;
	int turn_angle = 0;
	int speed =0;
	int IMU_angle_offest = 0;
	int IMU_theta = 0;
	bool turnhead_flag = false;
	bool check_anotherside_obs = false;

	//-------preturn----------
	int preturn_enable = 0;
	int preturn_speed = 0;
	int preturn_dir = 0;
	int preturn_theta = 0;
	int preturn_time = 0;

	
	/****************** ini parameter******************/
	
	////////////////////////////////////////function/////////////////////////////////////////////

	void GetParameter(const strategy::GetParameter &msg);
	void initparameterpath();
	void readparameter();
	int def_turn_angle();
	int def_speed();
	void printinfo();
	int IMU_Modify();
	void readpreturnparameter();
	void slope();
	float get_IMU();

};


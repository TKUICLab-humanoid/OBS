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

	//////////////////////////////////////new strategy parameter ////////////////////////////////////////////
	
	/***************subscribe deepmatrix parameter******************/

	int Dy = 0;
	float Dx = 0;
	string parameter_path = "N";
	float RD = 0;
	float LD = 0;

	/******************subscribe deepmatrix parameter******************/


	
	/*********************stategy_main parameter ********************/
	
	enum strategy_state
	{
		INIT,
		AVOID,
		TURNHEAD,
		REDDOOR
	};

	int continuous_angle_offset = 0;
	bool init_flag = true;
	float IMU_Value = 0;

	SensorMode IMU_continuous;
	strategy_state strategy_state;
	
	/*********************stategy_main parameter ********************/

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

	int maxspeed = 0;
	int minspeed = 0;
	
	int dangerous_distance = 0;
	int continuousValue_x = 0;
	int turn_angle = 0;
	int preturn_enable = 0;
	int preturn_speed = 0;
	int preturn_dir = 0;
	int preturn_theta = 0;
	int preturn_time = 0;

	
	/****************** ini parameter******************/
	
	////////////////////////////////////////function/////////////////////////////////////////////

	void GetDeepMatrix(const strategy::DeepMatrix &msg);
	void initparameterpath();
	void readparameter();
	int  def_turn_angle();
	void printinfo();
	void readpreturnparameter();


};


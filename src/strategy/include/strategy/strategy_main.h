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

	enum strategy_state
	{
		INIT,
		AVOID,
		TURNHEAD,
		REDDOOR,
		CRAWL
	};

	//////////////////////////////////////new strategy parameter ////////////////////////////////////////////
	
	/***************subscribe deepmatrix parameter******************/

	int nearest_distance_y = 0;
	float x_boundary = 0;
	string parameter_path = "N";

	/******************subscribe deepmatrix parameter******************/


	
	/*********************stategy_main parameter ********************/

	strategy_state strategy_state;
	SensorMode IMU_continuous;
	int continous_angle_offest = 0;

	
	/*********************stategy_main parameter ********************/

	/******************ini  parameter******************/

	int dirdata[100];
	int continuous_angle_offset = 0;

	//-------preturn----------
	int preturn_enable = 0;
	int preturn_dir = 0;
	int preturn_time = 0;
	int preturn_speed = 0; ;
	int preturn_theta = 0;
	
	/****************** ini parameter******************/
	
	////////////////////////////////////////function/////////////////////////////////////////////

	void GetDeepMatrix(const strategy::DeepMatrix &msg);
	void initparameterpath();
	void readparameter();
	void printinfo();
	void load_preturn_txt();

};

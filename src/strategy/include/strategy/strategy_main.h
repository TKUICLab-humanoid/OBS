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
	
	//////////subscribe deepmatrix parameter/////////////////
	int nearest_distance_y = 0;

	float x_avg_to_boundary = 0;
	//////////subscribe deepmatrix parameter/////////////////


	string parameter_path = "N";

	////////////////////////////////////////function/////////////////////////////////////////////

	void GetDeepMatrix(const strategy::DeepMatrix &msg);
	void initparameterpath();

};


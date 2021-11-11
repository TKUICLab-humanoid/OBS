#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <stdio.h>
#include <std_msgs/String.h>
#include "tku_libs/strategy_info.h"
#include "tku_libs/TKU_tool.h"
#include "tku_libs/RosCommunication.h"
//////////////////////////////////////////////////////////

#include <stdlib.h>
#include "strategy/GetParameter.h"
#include <sys/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
///////////////////////////////////// new /////////////////////////////////
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <highgui.h>


#include <string>
#include <fstream>
#include <sstream>

#define IMAGEHEIGHT 240
#define IMAGEWIDTH 320

using namespace std;

class OBSimage
{
public:
	OBSimage(ros::NodeHandle &nh)
	{
		image_transport::ImageTransport it(nh);
		pub_colormodel = it.advertise("final_image", 1);
		GetParameter_Publish = nh.advertise<strategy::GetParameter>("/strategy/GetParameter_Topic", 1000);
		strategy_info = StrategyInfoInstance::getInstance();
		tool = ToolInstance::getInstance();
		ros_com = RosCommunicationInstance::getInstance();
	};
	~OBSimage(){};

	/****************************** function *************************************/
	void strategymain();
	void ImageInfo();
	void INIT_parameter();
	void SlopeCalculate();
	/****************************** parameter *************************************/
	//int Focus_Matrix[32] = { 3, 3, 4, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 15, 16, 16, 16, 16, 15, 15, 13, 11, 10, 9, 8, 7, 6, 5, 4, 4, 3, 3};
	//int Focus_Matrix[32] = { 5, 5, 6, 7, 8, 8, 9, 9, 10, 10, 11, 13, 15, 15, 16, 16, 16, 16, 15, 15, 13, 11, 10, 10, 9, 9, 8, 8, 7, 6, 5, 5};
	// int Focus_Matrix[32] = { 7, 7, 8, 8, 8, 10, 10, 11, 11, 12, 12, 13, 15, 15, 16, 16, 16, 16, 15, 15, 13, 12, 12, 11, 11, 10, 10, 8, 8, 8, 7, 7};
	//int Focus_Matrix[32] = { 13, 13, 13, 13, 14, 14, 14, 14, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 14, 14, 14, 14, 13, 13, 13, 13};
	//int Focus_Matrix[32] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
	//int Focus_Matrix[32] = { 11, 11, 11, 11, 12, 12, 12, 12, 13, 13, 13, 13, 15, 15, 16, 16, 16, 16, 15, 15, 13, 13, 13, 13, 12, 12, 12, 12, 11, 11, 11, 11};
	int Focus_Matrix[32] = { 7, 7, 7, 7, 9, 9, 9, 9, 11, 11, 11, 11, 12, 12, 13, 13, 13, 13, 12, 12, 11, 11, 11, 11, 9, 9, 9, 9, 7, 7, 7, 7};
	//int Focus_Matrix[32] = { 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 19, 20, 20, 20, 20, 19, 19, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18};

	int Deep_Matrix[32];
	int Filter_Matrix[32];

	int WR = 0;		//
	int WL = 0;		//

	float Xc = 0;	//the center of obstacle in focus matrix
	int Xb = 0;		//compare WR & WL,setting boundary 

	int Dy = 0;		//min of deepmatrix
	float Dx = 0;	//distance between Xc and Xb

	int Xc_count = 0;	//calculate number of obstacle in focus matrix
	int Xi_sum = 0;	//sum of obstacle in focus matrix x-axis value

	int W_R = 0;
	int W_L = 0;

	//0905++++
	int RD = 0;		//IN reddoor,Right Value
	int LD = 0;		//IN reddoor,Left Value


	int L_XMAX = 0;		//IN reddoor,單塊藍色權重(左藍XMAX)
	int R_XMIN = 0;		//IN reddoor,單塊藍色權重(右藍XMIN)

	int XMax_one = 0;			//IN reddoor,收齊所有xmax && xmin
	int XMin_one = 0;
	int XMin_two = 0;
	int XMax_two = 0;

	int LeftblueOBS_XMax = 0;		//IN reddoor,left side blue obs (XMax)
	int RightblueOBS_XMin= 0;		//IN reddoor,right side blue obs (XMin)

	float slope_avg = 0;	//IN reddoor,slpoe for RED_DOOR

	bool in_reddoor_flag = false;
	bool b_obs_flag = false;
	bool y_obs_flag = false;
	int l_center_Dy = 0;
	int r_center_Dy = 0;
	int center_Dy = 0;
	int Deep_sum= 0;
	bool one_b_flag = false;
	bool two_b_flag = false;
	//0905++++

	unsigned char *rValue, *gValue, *bValue;

	sensor_msgs::ImagePtr msg_compressimage;
    cv::Mat publish_image;
	image_transport::Publisher pub_colormodel;

	RosCommunicationInstance *ros_com;
	ToolInstance *tool;
	StrategyInfoInstance *strategy_info;

	ros::Publisher GetParameter_Publish;


	strategy::GetParameter getparameter_parameter;	


};

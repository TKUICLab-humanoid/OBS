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
#include "strategy/DeepMatrix.h"
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
		DeepMatrix_Publish = nh.advertise<strategy::DeepMatrix>("/strategy/DeepMatrix_Topic", 1000);
		strategy_info = StrategyInfoInstance::getInstance();
		tool = ToolInstance::getInstance();
		ros_com = RosCommunicationInstance::getInstance();
	};
	~OBSimage(){};
	void strategymain();

	int DeepMatrix_cnt[32];
	int color_cnt;
	bool blue_flag = false;
	float color_flag;
	unsigned char *rValue, *gValue, *bValue;

	sensor_msgs::ImagePtr msg_compressimage;
    cv::Mat publish_image;
	image_transport::Publisher pub_colormodel;

	RosCommunicationInstance *ros_com;
	ToolInstance *tool;
	StrategyInfoInstance *strategy_info;

	

	ros::Publisher DeepMatrix_Publish;


	strategy::DeepMatrix deepmatrix;	
};

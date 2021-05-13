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

	int Focus_matrix[32] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1};
	int DeepMatrix_cnt[32];
	int FilterMatrix[32];

	int WR = 0;		//右權重值
	int WL = 0;		//左權重值

	float Xc = 0;	//在焦點矩陣內的x座標平均值
	int Xb = 0;		//權重值比較後的邊界設定
	int Xc_count = 0;
	int Xi_sum	=	0;

	int Dy = 0;
	float Dx = 0;
	
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

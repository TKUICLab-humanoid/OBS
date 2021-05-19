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

#define IMAGEHEIGHT 24
#define IMAGEWIDTH 32
#define DeepMatrixSize 32


int FocusMatrix[32] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10}; //攝影機內之焦點矩陣
//int LeftMove[32]  = { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  2,  3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 21};
//int RightMove[32] = {21, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//int Left[32]= {1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4};
//int Right[32]={4,4,4,4,4,4,4,4,3,3,3,3,3,3,3,3,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1};

int DeepMatrix[32];
int FilterMatrix[32];
int Dy = 0;     //D矩陣內最靠近機器人的值
int WR = 0;	//右權重
int WL = 0;	//左權重
int WL_sigma = 0;	//左權重相加
int WR_sigma = 0;	//右權重相加
int Xb = 0;	//判斷後的邊界設定	
int Xc_cnt = 0;	//計數
int Xc_n = 0;	//總和相加
int Xc = 0;	//焦點矩陣內x的平均值
float Dx = 0;	//OBS_avg到邊線的值


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

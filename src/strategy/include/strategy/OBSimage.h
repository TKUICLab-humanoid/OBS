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
#include <tuple>
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

class OBSImageAlgorithm
{
public:
	OBSImageAlgorithm()
	{
            value128 = cv::Mat(240, 320, CV_8UC1, cv::Scalar(128));
            value0 = cv::Mat(240, 320, CV_8UC1, cv::Scalar(0));
            unFocus_area = cv::Mat(24, 32, CV_8UC1, cv::Scalar(0));
            for(int c = 0; c < 32; c++)
            {
                for(int r = 0; r < 23 - Focus_area[c]; r++)
                {
                    *(unFocus_area.data + (r * 32 + c)) = 255;
                }
            }
	};
	~OBSImageAlgorithm(){};

	int Focus_area[32] = {1, 2, 6, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13, 13, 14, 14, 13, 13, 13, 12, 12, 11, 11, 10, 10, 9, 9, 8, 6, 2, 1};


        // For debug
	cv::Mat unFocus_area;
        // For debug
        //
	cv::Mat value128;
        cv::Mat value0;

	cv::Mat matB, matG, matR;

	void split_img_channels(cv::Mat bgr_img);
	cv::Mat labelImg2Binary(cv::Mat matBValue, cv::Mat matGValue, cv::Mat matRValue);
	cv::Mat compress_image(cv::Mat ori_img);

	std::vector<int> calc_deep_matrix(cv::Mat compress_img);
	std::vector<int> calc_obs_in_area_array(std::vector<int> deep_matrix);
	std::tuple<int, int> calc_wl_wr(std::vector<int> deepMatrix, std::vector<int> obs_inArea);

	std::tuple<int, int> calc_dx_dy(int wl, int wr, std::vector<int> obs_inArea);

	std::tuple<int, int> excute(cv::Mat label_img);
	
};

class OBSimage
{
public:
	OBSimage(ros::NodeHandle &nh)
	{
		//image_transport::ImageTransport it(nh);
		//pub_colormodel = it.advertise("final_image", 1);
		DeepMatrix_Publish = nh.advertise<strategy::DeepMatrix>("/strategy/DeepMatrix_Topic", 1000);
		strategy_info = StrategyInfoInstance::getInstance();
		//tool = ToolInstance::getInstance();
		ros_com = RosCommunicationInstance::getInstance();
	};
	~OBSimage(){};
	void strategymain();

	strategy::DeepMatrix msg_distance;

	//sensor_msgs::ImagePtr msg_compressimage;
    //cv::Mat publish_image;
	//image_transport::Publisher pub_colormodel;

	RosCommunicationInstance *ros_com;
	//ToolInstance *tool;
	StrategyInfoInstance *strategy_info;

	ros::Publisher DeepMatrix_Publish;
	strategy::DeepMatrix deepmatrix;	

	OBSImageAlgorithm obsImageAlgorithm;
};


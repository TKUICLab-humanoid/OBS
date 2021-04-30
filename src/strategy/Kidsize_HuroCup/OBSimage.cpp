#include "strategy/OBSimage.h"
#include <time.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "OBSimage");
	ros::NodeHandle nh;
	OBSimage OBSimage(nh);

	ros::Rate loop_rate(30);
	

	while (nh.ok()) 
	{
		OBSimage.strategymain();
		ros::spinOnce();
		loop_rate.sleep();
        
	}
	return 0;
}

void OBSimage::strategymain()
{
    clock_t start, end;
    cv::Mat mask128 (240,320,CV_8UC1, cv::Scalar(128));
    cv::Mat mask0 (240,320,CV_8UC1, cv::Scalar(0));
    std::vector<cv::Mat> rgbChannels(3);
    cv::Mat roi;
    cv::namedWindow("opencv resize", cv::WINDOW_NORMAL);
    cv::namedWindow("10*10", cv::WINDOW_NORMAL);
    //int DeepM_D[32]={0};
    std::vector<int> DeepM_D(32,0);
    //int Focus_area[32]={0};
    int Focus_area[32]={1,2,6,8,9,9,10,10,11,11,12,12,13,13,13,14,14,13,13,13,12,12,11,11,10,10,9,9,8,6,2,1};
    //int OBS_inArea[32]={0};
    std::vector<int> OBS_inArea(32,0);
    int turn_right=0;
    int turn_left=0;
    int dx = 23; 
    int dy = 0;
    int yb = 0;
    int yc = 0;
    int cnt = 0;

	if(strategy_info->getStrategyStart())
	{	
//===============transfer image from 320*240 to 32*24=======================
        cv::Mat ori_image = strategy_info->cvimg->image;
        start = clock();
        cv::Mat image;
        cv::resize(ori_image, image, cv::Size(32, 24));
        end = clock();
        // ROS_INFO("opencv resize time: %f", double(end-start)/CLOCKS_PER_SEC);
        cv::imshow("opencv resize", image);

        
        start = clock();
        cv::Mat compress_img = cv::Mat::zeros(24,32,CV_8UC1);

        cv::split(ori_image, rgbChannels);
        cv::Mat B = rgbChannels[0];
        cv::Mat G = rgbChannels[1];
        cv::Mat R = rgbChannels[2];

        cv::Mat maskB = B == mask128;
        cv::Mat maskG = G == mask0;
        cv::Mat maskR = R == mask128;

        cv::bitwise_and(maskB, maskG, maskB);
        cv::bitwise_and(maskB, maskR, maskB);

        for(int r = 0; r < 24; r++)
        {
            for (int c = 0; c < 32; c++)
            {
                roi = maskB(cv::Rect(c*10, r*10, 10, 10));
                int val = cv::countNonZero(roi);
                if(val > 20)
                {
                    *(compress_img.data + (r*32+c)) = 255;
                }
            }
        }

        end = clock();
        // ROS_INFO("10*10 time: %f", double(end-start)/CLOCKS_PER_SEC);
        cv::imshow("10*10", compress_img);
        cv::waitKey(1);
    //======================deep matrix D========================================          
        for(int c = 0; c < 32; c++)
        {
            
            for(int r = 23; r >= 0; r--)
            {
                if(*(compress_img.data + (r*32+c)))
                {
                    DeepM_D[c] = 23 - r;
                    break;
                }
                else
                {
                    DeepM_D[c] = 23;
                }
            }
        }
        // for(int i = 0; i < 32; i++)
        // {
        //     ROS_INFO("%d D: %d", i, DeepM_D[i]);
        // }         
    //=======================Focus area===========================================
        // for(int i = 0; i < 32; i++)
        // {
        //     if(i < 16)
        //     {
        //         Focus_area[i] = i + 1;
        //     }
        //     else
        //     {
        //         Focus_area[i] = 32 - i;
        //     }
        // }
        // for(int i = 0; i < 32; i++)
        // {
        //     ROS_INFO("%d F: %d", i, Focus_area[i]);
        // }
    //=======================OBS in Focus area=====================================  
        for(int i = 0; i < 32; i++)
        {
            if(Focus_area[i] >= DeepM_D[i])
            {
                OBS_inArea[i] = Focus_area[i] - DeepM_D[i];
            }
            else
            {
                OBS_inArea[i] = 0;
            }
        }
        // for(int i = 0; i < 32; i++)
        // {
        //     ROS_INFO("%d OBS_inArea: %d", i, OBS_inArea[i]);
        // }
    //=========================Calculate WL&WR=====================================
        for(int i = 0; i < 32; i++)
        {
            turn_right = turn_right + (32-i)*OBS_inArea[i];
            turn_left = turn_left + (i+1)*OBS_inArea[i];
            // ROS_INFO("%d turn_right: %d", i, turn_right);
            // ROS_INFO("%d turn_left: %d", i, turn_left);
        }
        // ROS_INFO("turn_right: %d", turn_right);
        // ROS_INFO("turn_left: %d", turn_left);
        // if(turn_right >=turn_left)
        // {
        //     ROS_INFO("turn_right: %d", turn_right);
        // }
        // else
        // {
        //     ROS_INFO("turn_left: %d", turn_left);
        // }
    //=========================Calculate Dx&Dy&Xc&Xb=====================================
        for(int i = 0; i < 32; i++)
        {
            if(OBS_inArea[i])
            {
                cnt++;
                yc += i;
            }
            if(DeepM_D[i] < dx)
            {
                dx = DeepM_D[i];
            }
        }
        if(cnt)
            yc = yc / cnt;

        if(turn_left > turn_right)
            yb = 31;
        else
            yb = 0;

        dy = yc - yb;

        msg_distance.dx = dx;
        msg_distance.dy = dy;

        DeepMatrix_Publish.publish(msg_distance);
    }
}

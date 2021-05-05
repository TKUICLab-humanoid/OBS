#include "strategy/OBSimage.h"
#include <time.h>
/*
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
*/

int main(int argc, char** argv)
{
	ros::init(argc, argv, "OBSimageAlgorithm");
	ros::NodeHandle nh;

    ros::Publisher DeepMatrix_Publish = nh.advertise<strategy::DeepMatrix>("/strategy/DeepMatrix_Topic", 1);

	RosCommunicationInstance *ros_com = RosCommunicationInstance::getInstance();
    StrategyInfoInstance *strategy_info = StrategyInfoInstance::getInstance();

	OBSImageAlgorithm obsImageAlgorithm;

	ros::Rate loop_rate(30);
	
    int dx = 0, dy = 0;
    strategy::DeepMatrix msg_distance;

	while (nh.ok()) 
	{

	    if(strategy_info->getStrategyStart())
	    {	
            cv::Mat label_image = strategy_info->cvimg->image;

            std::tie(dx, dy) = obsImageAlgorithm.excute(label_image);

            msg_distance.dx = dx;
            msg_distance.dy = dy;

            DeepMatrix_Publish.publish(msg_distance);
        }
        ros::spinOnce();
		loop_rate.sleep();        
	}
	return 0;
}
void OBSImageAlgorithm::split_img_channels(cv::Mat bgr_img)
{
    std::vector<cv::Mat> rgbChannels(3);
    cv::split(bgr_img, rgbChannels);

    matB = rgbChannels[0];
    matG = rgbChannels[1];
    matR = rgbChannels[2];
}

cv::Mat OBSImageAlgorithm::labelImg2Binary(cv::Mat matBValue, cv::Mat matGValue, cv::Mat matRValue)
{
    cv::Mat mat_binary;

    cv::Mat maskB = matB == matBValue;
    cv::Mat maskG = matG == matGValue;
    cv::Mat maskR = matR == matRValue;

    cv::bitwise_and(maskB, maskG, mat_binary);
    cv::bitwise_and(mat_binary, maskR, mat_binary);

    return mat_binary;
}

cv::Mat OBSImageAlgorithm::compress_image(cv::Mat ori_img)
{
    cv::Mat compress_img = cv::Mat::zeros(24, 32, CV_8UC1);

    cv::Mat binary_OBS;

    split_img_channels(ori_img);

    cv::Mat binary_blueObs = labelImg2Binary(value128, value0, value128);
    cv::Mat binary_yellowObs = labelImg2Binary(value128, value128, value0);

    cv::bitwise_or(binary_yellowObs, binary_blueObs, binary_OBS);

    cv::Mat roi;
    for (int r = 0; r < 24; r++)
    {
        for (int c = 0; c < 32; c++)
        {
            roi = binary_OBS(cv::Rect(c * 10, r * 10, 10, 10));
            int val = cv::countNonZero(roi);
            if (val > 20)
            {
                *(compress_img.data + (r * 32 + c)) = 255;
            }
        }
    }

    return compress_img;
}

std::vector<int> OBSImageAlgorithm::calc_deep_matrix(cv::Mat compress_img)
{
    std::vector<int> DeepM_D(32, 23);
    for (int c = 0; c < 32; c++)
    {

        for (int r = 23; r >= 0; r--)
        {
            if (*(compress_img.data + (r * 32 + c)))
            {
                DeepM_D[c] = 23 - r;
                break;
            }
        }
    }

    return DeepM_D;
}

std::vector<int> OBSImageAlgorithm::calc_obs_in_area_array(std::vector<int> deep_matrix)
{
    std::vector<int> OBS_inArea(32, 0);
    for (int i = 0; i < 32; i++)
    {
        OBS_inArea[i] = deep_matrix[i] - Focus_area[i];
    }
    return OBS_inArea;
}

std::tuple<int, int> OBSImageAlgorithm::calc_wl_wr(std::vector<int> deepMatrix, std::vector<int> obs_inArea)
{
    int turn_right = 0;
    int turn_left = 0;
    for (int i = 0; i < 32; i++)
    {
        if (obs_inArea[i] < 0)
        {
            turn_right = turn_right + (32 - i) * (-obs_inArea[i]);
            turn_left = turn_left + (i + 1) * (-obs_inArea[i]);
        }
    }
    if (turn_right)
        turn_right = turn_right + (23 - deepMatrix[0]);
    if (turn_left)
        turn_left = turn_left + (23 - deepMatrix[31]);

    return std::make_tuple(turn_left, turn_right);
}

std::tuple<int, int> OBSImageAlgorithm::calc_dx_dy(int wl, int wr, std::vector<int> obs_inArea)
{
    int dx = 23;
    int dy = 0;
    int yb = 0;
    int yc = 0;
    int cnt = 0;
    //Calculate Dx&Dy&Yc&Yb
    for (int i = 0; i < 32; i++)
    {
        if (obs_inArea[i] < 0)
        {
            cnt++;
            yc += i;
        }

        if (obs_inArea[i] < dx)
        {
            dx = obs_inArea[i];
        }
    }
    if (cnt)
        yc = yc / cnt;

    if (wl > wr)
        yb = 31;
    else
        yb = 0;

    dy = yc - yb;

    // ROS_INFO("WL: %d, WR: %d", wl, wr);
    // ROS_INFO("dx: %d, dy: %d", dx, dy);

    return std::make_tuple(dx, dy);
}

std::tuple<int, int> OBSImageAlgorithm::excute(cv::Mat label_img)
{
    int wl = 0, wr = 0;

    cv::Mat compress_img = compress_image(label_img);

    std::vector<int> deep_matrix = calc_deep_matrix(compress_img);

    std::vector<int> obs_inArea = calc_obs_in_area_array(deep_matrix);

    std::tie(wl, wr) = calc_wl_wr(deep_matrix, obs_inArea);

    return calc_dx_dy(wl, wr, obs_inArea);
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
        cv::Mat binary_blue;
        cv::Mat binary_yellow;
        cv::Mat binary_OBS;

        cv::split(ori_image, rgbChannels);
        cv::Mat B = rgbChannels[0];
        cv::Mat G = rgbChannels[1];
        cv::Mat R = rgbChannels[2];
//===============Blue=========================
        cv::Mat maskB = B == mask128;
        cv::Mat maskG = G == mask0;
        cv::Mat maskR = R == mask128;

        cv::bitwise_and(maskB, maskG, binary_blue);
        cv::bitwise_and(binary_blue, maskR, binary_blue);
//===============Yellow=======================
        maskG = G == mask128; //maskB is already mask128
        maskR = R == mask0;
        cv::bitwise_and(maskB, maskG, binary_yellow);
        cv::bitwise_and(binary_yellow, maskR, binary_yellow);
        cv::bitwise_or(binary_yellow, binary_blue, binary_OBS);



        for(int r = 0; r < 24; r++)
        {
            for (int c = 0; c < 32; c++)
            {
                roi = binary_OBS(cv::Rect(c*10, r*10, 10, 10));
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
            // if(Focus_area[i] >= DeepM_D[i])
            // {
            //     OBS_inArea[i] = Focus_area[i] - DeepM_D[i];
            // }
            // else
            // {
            //     OBS_inArea[i] = 0;
            // }
            OBS_inArea[i] = DeepM_D[i] - Focus_area[i];
        }
        
        // for(int i = 0; i < 32; i++)
        // {
        //     ROS_INFO("%d OBS_inArea: %d", i, OBS_inArea[i]);
        // }
    //=========================Calculate WL&WR=====================================
        for(int i = 0; i < 32; i++)
        {
            if(OBS_inArea[i] < 0)
            {
                turn_right = turn_right + (32-i)*(-OBS_inArea[i]);
                turn_left = turn_left + (i+1)*(-OBS_inArea[i]);
                // ROS_INFO("%d turn_right: %d", i, turn_right);
                // ROS_INFO("%d turn_left: %d", i, turn_left);
            }
        }
        if(turn_right)
            turn_right = turn_right + (23-DeepM_D[0]);
        if(turn_left)
            turn_left = turn_left + (23-DeepM_D[31]);
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
    //=========================Calculate Dx&Dy&Yc&Yb=====================================
        for(int i = 0; i < 32; i++)
        {
            if(OBS_inArea[i] < 0)
            {
                cnt++;
                yc += i;
            }

            if(OBS_inArea[i] < dx)
            {
                dx = OBS_inArea[i];
            }
        }
        if(cnt)
            yc = yc / cnt;

        if(turn_left > turn_right)
            yb = 31;
        else
            yb = 0;

        dy = yc - yb;

        // ROS_INFO("dx: %d, dy: %d", dx, dy);

        msg_distance.dx = dx;
        msg_distance.dy = dy;

        DeepMatrix_Publish.publish(msg_distance);
    }
}

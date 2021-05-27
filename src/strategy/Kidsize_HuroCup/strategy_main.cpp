#include "strategy/strategy_main.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OBSstrategy");
    ros::NodeHandle nh;
    KidsizeStrategy KidsizeStrategy(nh);

    ros::Rate loop_rate(30);

    KidsizeStrategy.initparameterpath();

    while (nh.ok())
    {
        KidsizeStrategy.strategymain();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void KidsizeStrategy::strategymain()
{
    
    if (strategy_info->getStrategyStart()) //策略指撥開啟
    {
        switch (m_state)
        {
        case P_INIT:
            printinfo();
            m_state_string = "P_INIT";
            readwalkinggait(); 
            load_dirtxt();

            /*if (!Continuous_flag) //起步步態
            {
                ros_com->sendBodySector(4); //動作磁區
                tool->Delay(1000);
                ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous); //ros_com->sendBodyAuto(-450, 0, 0,-3, WalkingMode::ContinuousStep,IMU_continuous);
                tool->Delay(500);
                Continuous_flag = true;
            }*/


        case P_DOOR: //紅門策略
            printinfo();

        case P_CRAWL: //爬行策略
            printinfo();
            m_state_string = "P_CRAWL";
            ROS_INFO("P_CRAWL___");
            break;

        case P_WALKINGGAIT: //步態參數及補償量
            printinfo();
            m_state_string = "P_WALKINGGAIT";
            ROS_INFO("P_WALKINGGAIT_________");

            switch (walking_state)
            {
                case continuousValue_Rt:
                    walking_state_string = "continousValue_Rt";
                    ROS_INFO("continousValue_Rt_________");
                    break;
                case continuousValue_Lt:
                    walking_state_string = "continousValue_Lt";
                    ROS_INFO("continousValue_Lt_______");
                default:
                    ROS_INFO("case default");
            }
        }
    }
    else //策略指撥關閉
    {
        /*if (stand_flag == true)
        {
            //ROS_INFO("handdown");
            if (Continuous_flag)
            {
                ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous); //關閉連續步態
                Continuous_flag = false;
                tool->Delay(1500);
            }
            stand_flag = false;
            ros_com->sendBodySector(5);
            tool->Delay(1000);
            ros_com->sendBodySector(29);
            tool->Delay(1000);
            first_cnt = 0;
            ROS_INFO("stop");
        }*/
        m_state = P_INIT;
        readwalkinggait();
    }
}

void KidsizeStrategy::initparameterpath()
{
    while (parameter_path == "N")
    {
        parameter_path = tool->getPackagePath("strategy");
    }
    printf("parameter_path is %s\n", parameter_path.c_str());
}
void KidsizeStrategy::load_dirtxt() //first_move讀檔之副函式
{
    fstream fin;
    string sTmp;
    char line[100];
    char path[200];
    strcpy(path, parameter_path.c_str());
    strcat(path, "/firstmove.ini");
    fin.open(path, ios::in);
    /*try
    {
        dirmap[0] = tool->readvalue(fin, "enable", 1);
        dirmap[1] = tool->readvalue(fin, "dir", 1);
        dirmap[2] = tool->readvalue(fin, "time", 1);
        fin.close();
    }*/
    /*catch (exception e)
    {
    }*/
}
void KidsizeStrategy::readwalkinggait() //步態參數之讀檔
{
    fstream fin;
    string sTmp;
    char line[100];
    char path[200];
    strcpy(path, parameter_path.c_str());
    strcat(path, "/WalkingGait.ini");
    fin.open(path, ios::in);
    char temp[100];
    try
    {
        fin.getline(temp, sizeof(temp));
        dirdata[30] = tool->readvalue(fin, "continuous_x_offset", 0);
        dirdata[31] = tool->readvalue(fin, "continuous_y_offset", 0);
        dirdata[32] = tool->readvalue(fin, "continuous_theta_offset", 0);
        dirdata[33] = tool->readvalue(fin, "continuous_x_offset_RIGHT", 0);
        dirdata[34] = tool->readvalue(fin, "continuous_y_offset_RIGHT", 0);
        dirdata[35] = tool->readvalue(fin, "continuous_theta_offset_RIGHT", 0);
        dirdata[36] = tool->readvalue(fin, "continuous_x_offset_LEFT", 0);
        dirdata[37] = tool->readvalue(fin, "continuous_y_offset_LEFT", 0);
        dirdata[38] = tool->readvalue(fin, "continuous_theta_offset_LEFT", 0);
        fin.close();
    }
    catch (exception e)
    {
    }
}
void KidsizeStrategy::GetDeepMatrix(const strategy::DeepMatrix &msg) //深度矩陣之副函式
{
        //DeepMatrixValue[i] = msg.DeepMatrix[i];
        dy = msg.dy;
        dx = msg.dx;

}
void KidsizeStrategy::printinfo()
{
    ROS_INFO("\n\n\n\n\n");
    //ROS_INFO("%s", m_state_string.c_str());
    /*if (zero_flag)
    {
        //walking_state_string="forward";
        ROS_INFO("zero_flag = true");
    }
    else*/
        ROS_INFO("zero_flag = false");
	
	//ROS_INFO("continous_angle_offest = %d",continous_angle_offest);
	//ROS_INFO("angle_offset = %d",angle_offset);

    ROS_INFO("%s",walking_state_string.c_str());
    //ROS_INFO("continuousValue_x = %5d", continuousValue_x);
    ROS_INFO("[30] = %5d [31] = %5d [32] = %5d", dirdata[30], dirdata[31], dirdata[32]);
    ROS_INFO("[33] = %5d [34] = %5d [35] = %5d", dirdata[33], dirdata[34], dirdata[35]);
    ROS_INFO("[36] = %5d [37] = %5d [38] = %5d", dirdata[36], dirdata[37], dirdata[38]);

    /*if (leftsidelinewarning)
        ROS_INFO("leftsidelinewarning = true");
    else
        ROS_INFO("leftsidelinewarning = false");
    if (rightsidelinewarning)
        ROS_INFO("rightsidelinewarning = true");
    else
        ROS_INFO("rightsidelinewarning = false");*/
    //ROS_INFO("true_RMoveValue = %d", true_RMoveValue);
    //ROS_INFO("true_LMoveValue = %d", true_LMoveValue);
    ROS_INFO("/////////////Red Door/////////////");
    //ROS_INFO("Red Door slope_avg = %f", slope_avg);
    /*if (Center_door)
        ROS_INFO("center_door = true");
    else
        ROS_INFO("center_door = false");
    if (Blue_obs_flag)
        ROS_INFO("Blue_obs_flag = true");
    else
        ROS_INFO("Blue_obs_flag = false");
    if (twentyflag)
        ROS_INFO("<20");
    else if (m_state == P_DOOR && twentyflag == false)
        ROS_INFO(">20");
    else
        ROS_INFO(" ");
    ROS_INFO("IMU = %f", strategy_info->getIMUValue().Yaw);
    if (first_move_flag)
        ROS_INFO("first_move_flag = true");
    else
        ROS_INFO("first_move_flag = false");
    if (first_act_flag)
        ROS_INFO("first_act_flag = true");
    else
        ROS_INFO("first_act_flag = false");*/
    //ROS_INFO("side line slope = %f", sidelineslope);
    //ROS_INFO("\n\n\n");
}
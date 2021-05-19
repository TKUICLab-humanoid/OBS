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
        readwalkinggait();
        /*ROS_INFO("Dx = %f",OBS_avg_to_SideLine);
        ROS_INFO("Dy = %d",Deep_min_y);
        ROS_INFO("WR = %d",Deep_WR);
        ROS_INFO("WL = %d",Deep_WL);
        ROS_INFO("Xc = %d",OBS_avg);
        ROS_INFO("Xb = %d",SideLine);*/

        ROS_INFO("X = %d,Y = %d,Z = %d",dirdata[0],dirdata[1],dirdata[2]);



        /*if (!Continuous_flag) //起步步態
            {
                ros_com->sendBodySector(4); //動作磁區
                tool->Delay(1000);
                ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous); //ros_com->sendBodyAuto(-450, 0, 0,-3, WalkingMode::ContinuousStep,IMU_continuous);
                tool->Delay(500);
                Continuous_flag = true;
            }
            ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1603, 300);
            tool->Delay(100);
            ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300); //頭部馬達刻度（左右）左正右負
            tool->Delay(100);*/
        ROS_INFO("gogogo");
    }
        
    else //策略指撥關閉
    {
        ROS_INFO("stopppppp");
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
        }
        m_state = P_INIT;
        readwalkinggait();*/
    }
}

/*void KidsizeStrategy::SlopeCalculate() //計算斜率之副函式
{
    //////////////////2//////////////////
    int compareObs_x = 1000000;
    int compareObs_ymax = 0;
    int Slope_base_obs = 0;
    bool Check_obs_flag = false;
    //////////////////2//////////////////
    bool Check_label_model_flag = true;
    rest_flag = false;
    int Check_Xmin = 160;
    int Check_Xmax = 160;
    int Check_Ymax = 0;
    int Check_Ymax_another_side = 0;
    int slope_rand[4];
    int slope_Y[4];
    float slope[3];
    slope_avg = 1000000.0;
    slope_avg_blue = 1000000.0;
    if (!in_reddoor_flag)
    {
        ///ROS_INFO("not in_reddoor_flag!!");
        for (int i = 0; i < strategy_info->color_mask_subject_cnts[2]; i++)
        {
            ///ROS_INFO("Size_blue = %d",strategy_info->color_mask_subject[2][i].size);
            if (strategy_info->color_mask_subject[2][i].size > 8000)
            {
                Check_obs_flag = true;
                ///ROS_INFO("Size_blue = %d",strategy_info->color_mask_subject[2][i].size);
                if (strategy_info->color_mask_subject[2][i].YMax > compareObs_ymax)
                {
                    Slope_base_obs = i;
                    compareObs_ymax = strategy_info->color_mask_subject[2][i].YMax;
                    ///ROS_INFO("Near the robot!!");
                    if (abs(160 - strategy_info->color_mask_subject[2][i].X) < compareObs_x)
                    {
                        ///ROS_INFO("Near the 160!!");
                        Slope_base_obs = i;
                        compareObs_x = abs(160 - strategy_info->color_mask_subject[2][i].X);
                    }
                }
            }
        }
        if (Check_obs_flag)
        {
            ///ROS_INFO("Check_obs_flag true!!");
            for (int j = 0; j < 4; j++)
            {
                int repeat_cnt = 0;
                int range = (strategy_info->color_mask_subject[2][Slope_base_obs].XMax - 7) - (strategy_info->color_mask_subject[2][Slope_base_obs].XMin + 7);
                slope_rand[j] = rand() % range + strategy_info->color_mask_subject[2][Slope_base_obs].XMin;
                while (1)
                {
                    if (repeat_cnt != j)
                    {
                        if (slope_rand[j] == slope_rand[repeat_cnt])
                        {
                            repeat_cnt = 0;
                            slope_rand[j] = rand() % range + strategy_info->color_mask_subject[2][Slope_base_obs].XMin;
                        }
                    }
                    else
                    {
                        break;
                    }
                    repeat_cnt++;
                }
            }
            for (int k = 0; k < 4; k++)
            {
                bool flag = true;
                int Xmax = strategy_info->color_mask_subject[2][Slope_base_obs].XMax;
                int Ymax = strategy_info->color_mask_subject[2][Slope_base_obs].YMax;
                int cnt = 0;
                int labelcnt;
                while (flag)
                {
                    labelcnt = 320 * (Ymax - cnt + 1) + slope_rand[k];
                    if (strategy_info->label_model[labelcnt] == 0x04)
                    {
                        for (int a = 1; a < 4; a++)
                        {
                            if (strategy_info->label_model[labelcnt - 320 * a] != 0x04)
                            {
                                Check_label_model_flag = false;
                                break;
                            }
                        }
                        if (Check_label_model_flag)
                        {
                            slope_Y[k] = Ymax - cnt;
                            flag = false;
                        }
                        else
                        {
                            Check_label_model_flag = true;
                        }
                    }
                    if ((cnt + 1) > Ymax)
                    {
                        slope_Y[k] = strategy_info->color_mask_subject[2][Slope_base_obs].YMin;
                        flag = false;
                    }
                    else
                    {
                        cnt++;
                    }
                }
            }
            slope[0] = float(slope_Y[1] - slope_Y[0]) / float(slope_rand[1] - slope_rand[0]);
            slope[1] = float(slope_Y[2] - slope_Y[1]) / float(slope_rand[2] - slope_rand[1]);
            slope[2] = float(slope_Y[3] - slope_Y[2]) / float(slope_rand[3] - slope_rand[2]);
            slope_avg_blue = (slope[0] + slope[1] + slope[2]) / 3;
        }
        else
        {
            continous_angle_offest = 0;
        }
    }
    else
    {
        for (int i = 0; i < strategy_info->color_mask_subject_cnts[5]; i++)
        {
            if (strategy_info->color_mask_subject[5][i].size > 3000)
            {
                for (int j = 0; j < 4; j++)
                {
                    int repeat_cnt = 0;
                    int range = (strategy_info->color_mask_subject[5][i].XMax - 10) - (strategy_info->color_mask_subject[5][i].XMin + 10);
                    slope_rand[j] = rand() % range + strategy_info->color_mask_subject[5][i].XMin;
                    while (1)
                    {
                        if (repeat_cnt != j)
                        {
                            if (slope_rand[j] == slope_rand[repeat_cnt])
                            {
                                repeat_cnt = 0;
                                slope_rand[j] = rand() % range + strategy_info->color_mask_subject[5][i].XMin;
                            }
                        }
                        else
                        {
                            break;
                        }
                        repeat_cnt++;
                    }
                }
                for (int k = 0; k < 4; k++)
                {
                    bool flag = true;
                    int Xmax = strategy_info->color_mask_subject[5][i].XMax;
                    int Ymax = strategy_info->color_mask_subject[5][i].YMax;
                    int cnt = 0;
                    int labelcnt;
                    while (flag)
                    {
                        labelcnt = 320 * (Ymax - cnt + 1) + slope_rand[k];
                        if (strategy_info->label_model[labelcnt] == 0x20)
                        {
                            for (int a = 1; a < 4; a++)
                            {
                                if (strategy_info->label_model[labelcnt - 320 * a] != 0x20)
                                {
                                    Check_label_model_flag = false;
                                    break;
                                }
                            }
                            if (Check_label_model_flag)
                            {
                                slope_Y[k] = Ymax - cnt;
                                flag = false;
                            }
                            else
                            {
                                Check_label_model_flag = true;
                            }
                        }
                        if ((cnt + 1) > Ymax)
                        {
                            slope_Y[k] = strategy_info->color_mask_subject[5][i].YMin;
                            flag = false;
                        }
                        else
                        {
                            cnt++;
                        }
                    }
                }
                slope[0] = float(slope_Y[1] - slope_Y[0]) / float(slope_rand[1] - slope_rand[0]);
                slope[1] = float(slope_Y[2] - slope_Y[1]) / float(slope_rand[2] - slope_rand[1]);
                slope[2] = float(slope_Y[3] - slope_Y[2]) / float(slope_rand[3] - slope_rand[2]);
                slope_avg = (slope[0] + slope[1] + slope[2]) / 3;
                break;
            }
        }
    }
    if (abs(slope_avg) <= 0.05)
    {
        face_to_door = true;
    }
    else
    {
        face_to_door = false;
    }
}
void KidsizeStrategy::IMUSlope()
{
    if (Continuous_flag)
    {
        if (4 < strategy_info->getIMUValue().Yaw && strategy_info->getIMUValue().Yaw <= 180)
        {
            IMU_slope = strategy_info->getIMUValue().Yaw;
        }
        else if (-4 > strategy_info->getIMUValue().Yaw && strategy_info->getIMUValue().Yaw >= -179)
        {
            IMU_slope = strategy_info->getIMUValue().Yaw;
        }
    }
    else
    {
        if (20 < strategy_info->getIMUValue().Yaw && strategy_info->getIMUValue().Yaw <= 180)
        {
            IMU_slope = strategy_info->getIMUValue().Yaw;
        }
        else if (-20 > strategy_info->getIMUValue().Yaw && strategy_info->getIMUValue().Yaw >= -179)
        {
            IMU_slope = strategy_info->getIMUValue().Yaw;
        }
    }
}*/

void KidsizeStrategy::initparameterpath()
{
    while (parameter_path == "N")
    {
        parameter_path = tool->getPackagePath("strategy");
    }
    printf("parameter_path is %s\n", parameter_path.c_str());
}
/*void KidsizeStrategy::load_dirtxt() //first_move讀檔之副函式
{
    fstream fin;
    string sTmp;
    char line[100];
    char path[200];
    strcpy(path, parameter_path.c_str());
    strcat(path, "/firstmove.ini");
    fin.open(path, ios::in);
    try
    {
        dirmap[0] = tool->readvalue(fin, "enable", 1);
        dirmap[1] = tool->readvalue(fin, "dir", 1);
        dirmap[2] = tool->readvalue(fin, "time", 1);
        fin.close();
    }
    catch (exception e)
    {
    }
}*/
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
        dirdata[0] = tool->readvalue(fin, "continuous_forword_X", 0);
        dirdata[1] = tool->readvalue(fin, "continuous_forword_Y", 0);
        dirdata[2] = tool->readvalue(fin, "continuous_forword_T", 0);
        dirdata[3] = tool->readvalue(fin, "continuous_TurnRight_X", 0);
        dirdata[4] = tool->readvalue(fin, "continuous_TurnRight_Y", 0);
        dirdata[5] = tool->readvalue(fin, "continuous_TurnRight_T", 0);
        dirdata[6] = tool->readvalue(fin, "continuous_TurnLeft_X", 0);
        dirdata[7] = tool->readvalue(fin, "continuous_TurnLeft_Y", 0);
        dirdata[8] = tool->readvalue(fin, "continuous_TurnLeft_T", 0);
        fin.close();
    }
    catch (exception e)
    {
    }
}
void KidsizeStrategy::GetDeepMatrix(const strategy::DeepMatrix &msg) //深度矩陣之副函式
{
    for (int i = 0; i < 32; i++)
    {
        //DeepMatrixValue[i] = msg.DeepMatrix[i];
        OBS_avg_to_SideLine = msg.Dx;
        Deep_min_y = msg.Dy;
        Deep_WL = msg.WL;
        Deep_WR = msg.WR;
        OBS_avg = msg.Xc;
        SideLine = msg.Xb;
    }
}


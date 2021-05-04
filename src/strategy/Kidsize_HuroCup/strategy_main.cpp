#include "strategy/strategy_main.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OBSstrategy");
    ros::NodeHandle nh;
    KidsizeStrategy KidsizeStrategy(nh);

    ros::Rate loop_rate(2);

    KidsizeStrategy.initparameterpath();
    KidsizeStrategy.load_OBS_param();

    while (nh.ok())
    {
        KidsizeStrategy.strategymain();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void KidsizeStrategy::calc_Forward()
{
    int x_limit;
    if(dx <= -10)
    {
        x_limit = _forwardParam.stop;
        ROS_INFO("no go");       
    }
    else if(dx <= 0)
    {  
        x_limit = _forwardParam.small;
        ROS_INFO("go");
    }
    else
    {
        x_limit = _forwardParam.big;
        ROS_INFO("go go");
    }
    if(x_limit > _walkingParam.x)
    {
        _walkingParam.x += 100;
    }
    else if(x_limit < _walkingParam.x)
    {
        _walkingParam.x -= 100;
    }
}
void KidsizeStrategy::calc_Turn()
{
    int y_limit;
    if(dy <= -13)
    {
        y_limit = _turnParam.left_big;
        ROS_INFO("very left");
    }    
    else if (dy <= -5)
    {
        y_limit = _turnParam.left_small;
        ROS_INFO("normal left");
    }  
    else if (dy <= 5)
    {
        y_limit = _turnParam.no_turn;
        ROS_INFO("no turn");
    } 
    else if (dy <= 13)
    {
        y_limit = _turnParam.right_small;
        ROS_INFO("normal right");
    }   
    else
    {
        y_limit = _turnParam.right_big;
        ROS_INFO("very right");
    }
    if(y_limit > _walkingParam.theta)
    {
        _walkingParam.theta += 1;
    }
    else if(y_limit < _walkingParam.theta)
    {
        _walkingParam.theta -= 1;
    }  
}
void KidsizeStrategy::load_OBS_param()
{
    fstream fin;
    string sTmp;
    char line[100];
    char path[200];
    strcpy(path, parameter_path.c_str());
    strcat(path, "/OBS_param.ini");
    fin.open(path, ios::in);
    char temp[100];
    try
    {
        fin.getline(temp, sizeof(temp));
        _forwardParam.small = tool->readvalue(fin, "forward_small", 0);
        _forwardParam.big = tool->readvalue(fin, "forward_big", 0);
        _forwardParam.stop = tool->readvalue(fin, "forward_stop", 0);
        _turnParam.left_small = tool->readvalue(fin, "left_small", 0);
        _turnParam.left_big = tool->readvalue(fin, "left_big", 0);
        _turnParam.no_turn = tool->readvalue(fin, "no_turn", 0);
        _turnParam.right_small = tool->readvalue(fin, "right_small", 0);
        _turnParam.right_big = tool->readvalue(fin, "right_big", 0);
        fin.getline(temp, sizeof(temp));
        _walkingParam.x = tool->readvalue(fin, "walking_x_offset", 0);
        _walkingParam.y = tool->readvalue(fin, "walking_y_offset", 0);
        _walkingParam.theta = tool->readvalue(fin, "walking_theta_offset", 0);
        _forwardParam.stop = _walkingParam.x;
        _turnParam.no_turn = _walkingParam.theta;
        fin.close();
    }
    catch (exception e)
    {
    }
}
void KidsizeStrategy::strategymain()
{
    if (strategy_info->getStrategyStart()) //策略指撥開啟
    {
        if(!walking)
        {
            load_OBS_param();
            ros_com->sendBodySector(4);
            tool->Delay(1000);
            ros_com->sendBodyAuto(_walkingParam.x, _walkingParam.y, 0,_walkingParam.theta, WalkingMode::ContinuousStep, IMU_continuous);
            walking = true;
        }
        calc_Forward();
        calc_Turn();
        ros_com->sendContinuousValue(_walkingParam.x, _walkingParam.y, 0, _walkingParam.theta, IMU_continuous);
        ROS_INFO("_walkingParam.x: %d, _walkingParam.theta: %d", _walkingParam.x, _walkingParam.theta);
    }
    else //策略指撥關閉
    {
        _walkingParam.x = 0;
        _walkingParam.y  = 0;
        _walkingParam.theta = 0;
        if (stand_flag == true)
        {
            //ROS_INFO("handdown");
            if (walking)
            {
                ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous); //關閉連續步態
                walking = false;
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
    }
}

void KidsizeStrategy::SlopeCalculate() //計算斜率之副函式
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
}
void KidsizeStrategy::FaceToFinialLineFun() //正對終點方向修正的步態補償之副函式
{
    if (IMU_slope < 0)
    {
        if (abs(IMU_slope) >= 15)
        {
            continous_angle_offest = 4;
        }
        else if (abs(IMU_slope) >= 10 && abs(IMU_slope) < 15)
        {
            continous_angle_offest = 2;
        }
        else if (abs(IMU_slope) > 4 && abs(IMU_slope) < 10)
        {
            continous_angle_offest = 1;
        }
        else
        {
            continous_angle_offest = 0;
        }
    }
    else
    {
        if (abs(IMU_slope) >= 15)
        {
            continous_angle_offest = -4;
        }
        else if (abs(IMU_slope) >= 10 && abs(IMU_slope) < 15)
        {
            continous_angle_offest = -2;
        }
        else if (abs(IMU_slope) > 4 && abs(IMU_slope) < 10)
        {
            continous_angle_offest = -1;
        }
        else
        {
            continous_angle_offest = 0;
        }
    }
}

void KidsizeStrategy::traverse() //正對障終點方向修正的步態補償之副函式
{
    if (IMU_slope < 0)
    {
        if (abs(IMU_slope) > 15)
        {
            continous_angle_offest = 4;
        }
        else if (abs(IMU_slope) > 10 && abs(IMU_slope) <= 15)
        {
            continous_angle_offest = 3;
        }
        else if (abs(IMU_slope) > 4 && abs(IMU_slope) <= 10)
        {
            continous_angle_offest = 2;
        }
        else
        {
            continous_angle_offest = 0;
        }
    }
    else
    {
        if (abs(IMU_slope) > 15)
        {
            continous_angle_offest = -4;
        }
        else if (abs(IMU_slope) > 10 && abs(IMU_slope) <= 15)
        {
            continous_angle_offest = -3;
        }
        else if (abs(IMU_slope) > 4 && abs(IMU_slope) <= 10)
        {
            continous_angle_offest = -2;
        }
        else
        {
            continous_angle_offest = 0;
        }
    }
}

void KidsizeStrategy::FaceToObsFun() //正對障礙物修正之副函式
{
    if (slope_avg_blue < 0)
    {
        if (abs(slope_avg_blue) > 0.22 && abs(slope_avg_blue) < 0.3)
        {
            continous_angle_offest = 4;
        }
        else if (abs(slope_avg_blue) > 0.13 && abs(slope_avg_blue) < 0.21)
        {
            continous_angle_offest = 3;
        }
        else
        {
            continous_angle_offest = 0;
        }
    }
    else
    {
        if (abs(slope_avg_blue) > 0.22 && abs(slope_avg_blue) < 0.3)
        {
            continous_angle_offest = -4;
        }
        else if (abs(slope_avg_blue) > 0.13 && abs(slope_avg_blue) < 0.21)
        {
            continous_angle_offest = -3;
        }
        else
        {
            continous_angle_offest = 0;
        }
    }
}
void KidsizeStrategy::facetodoorfun() //正對紅門修正之副函式
{
    if (m_state == P_CRAWL)
        pcrawl_flag = true;
    else
        pcrawl_flag = false;

    if (first_enter_door)
    {
        if (slope_avg < 0.5 && slope_avg >= 0.25)
        {
            walking_state = continuousValue_Rt;
            m_state = P_WALKINGGAIT;
        }
        else if (slope_avg > -0.5 && slope_avg <= -0.25)
        {
            walking_state = continuousValue_Lt;
            m_state = P_WALKINGGAIT;
        }

        else if (slope_avg > 0 && slope_avg < 0.25)
        {
            walking_state = continuousValue_Rt;
            m_state = P_WALKINGGAIT;
            first_enter_door = false;
        }
        else if (slope_avg < 0 && slope_avg > -0.25)
        {
            walking_state = continuousValue_Lt;
            m_state = P_WALKINGGAIT;
            first_enter_door = false;
        }
        else if (slope_avg < 1 && slope_avg >= 0.5)
        {
            walking_state = continuousValue_R2t;
            m_state = P_WALKINGGAIT;
        }
        else if (slope_avg > -1 && slope_avg <= -0.5)
        {
            walking_state = continuousValue_L2t;
            m_state = P_WALKINGGAIT;
        }
    }
    else ////first_enter_door為false
    {
        if (slope_avg < 0.5 && slope_avg > 0)
        {
            walking_state = continuousValue_Rt;
            m_state = P_WALKINGGAIT;
        }
        else if (slope_avg > -0.5 && slope_avg < 0)
        {
            walking_state = continuousValue_Lt;
            m_state = P_WALKINGGAIT;
        }
        else if (slope_avg < 1 && slope_avg >= 0.5)
        {
            walking_state = continuousValue_R2t;
            m_state = P_WALKINGGAIT;
        }
        else if (slope_avg > -1 && slope_avg <= -0.5)
        {
            walking_state = continuousValue_L2t;
            m_state = P_WALKINGGAIT;
        }
    }
}
void KidsizeStrategy::give_angle()
{
    if (slope_avg < 0.5 && slope_avg >= 0.25)
    {
        continous_angle_offest = -2;
    }
    else if (slope_avg > -0.5 && slope_avg <= -0.25)
    {
        continous_angle_offest = 2;
    }
    else if (slope_avg < 0.25 && slope_avg >= 0.15)
    {
        continous_angle_offest = -1;
        pcrawl_flag = true;
    }
    else if (slope_avg > -0.25 && slope_avg <= -0.15)
    {
        continous_angle_offest = 1;
        pcrawl_flag = true;
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
        dirdata[0] = tool->readvalue(fin, "RMOVE_FASTX", 0);
        dirdata[1] = tool->readvalue(fin, "RMOVE_FASTY", 0);
        dirdata[2] = tool->readvalue(fin, "RMOVE_FASTT", 0);
        dirdata[3] = tool->readvalue(fin, "LMOVE_FASTX", 0);
        dirdata[4] = tool->readvalue(fin, "LMOVE_FASTY", 0);
        dirdata[5] = tool->readvalue(fin, "LMOVE_FASTT", 0);
        dirdata[6] = tool->readvalue(fin, "RMOVE_SLOWX", 0);
        dirdata[7] = tool->readvalue(fin, "RMOVE_SLOWY", 0);
        dirdata[8] = tool->readvalue(fin, "RMOVE_SLOWT", 0);
        dirdata[9] = tool->readvalue(fin, "LMOVE_SLOWX", 0);
        dirdata[10] = tool->readvalue(fin, "LMOVE_SLOWY", 0);
        dirdata[11] = tool->readvalue(fin, "LMOVE_SLOWT", 0);
        dirdata[12] = tool->readvalue(fin, "WC_BIGLEFTX", 0);
        dirdata[13] = tool->readvalue(fin, "WC_BIGLEFTY", 0);
        dirdata[14] = tool->readvalue(fin, "WC_BIGLEFTT", 0);
        dirdata[15] = tool->readvalue(fin, "WC_MIDLEFTX", 0);
        dirdata[16] = tool->readvalue(fin, "WC_MIDLEFTY", 0);
        dirdata[17] = tool->readvalue(fin, "WC_MIDLEFTT", 0);
        dirdata[18] = tool->readvalue(fin, "WC_LEFTX", 0);
        dirdata[19] = tool->readvalue(fin, "WC_LEFTY", 0);
        dirdata[20] = tool->readvalue(fin, "WC_LEFTT", 0);
        dirdata[21] = tool->readvalue(fin, "WC_BIGRIGHTX", 0);
        dirdata[22] = tool->readvalue(fin, "WC_BIGRIGHTY", 0);
        dirdata[23] = tool->readvalue(fin, "WC_BIGRIGHTT", 0);
        dirdata[24] = tool->readvalue(fin, "WC_MIDRIGHTX", 0);
        dirdata[25] = tool->readvalue(fin, "WC_MIDRIGHTY", 0);
        dirdata[26] = tool->readvalue(fin, "WC_MIDRIGHTT", 0);
        dirdata[27] = tool->readvalue(fin, "WC_RIGHTX", 0);
        dirdata[28] = tool->readvalue(fin, "WC_RIGHTY", 0);
        dirdata[29] = tool->readvalue(fin, "WC_RIGHTT", 0);
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
        fin.getline(temp, sizeof(temp));
        dirdata[39] = tool->readvalue(fin, "_continuous", 0);
        dirdata[40] = tool->readvalue(fin, "DIRmap_RIGHT_X", 0);
        dirdata[41] = tool->readvalue(fin, "DIRmap_RIGHT_Y", 0);
        dirdata[42] = tool->readvalue(fin, "DIRmap_RIGHT_T", 0);
        dirdata[43] = tool->readvalue(fin, "DIRmap_LEFT_X", 0);
        dirdata[44] = tool->readvalue(fin, "DIRmap_LEFT_Y", 0);
        dirdata[45] = tool->readvalue(fin, "DIRmap_LEFT_T", 0);
        dirdata[46] = tool->readvalue(fin, "IMU_single", 0);
        fin.close();
    }
    catch (exception e)
    {
    }
}
void KidsizeStrategy::GetDeepMatrix(const strategy::DeepMatrix &msg) //深度矩陣之副函式
{
    dx = msg.dx;
    dy = msg.dy;
}

void KidsizeStrategy::sideline()
{
    for (int i = 5; i < 27; i++)
    {
        if (DeepMatrixValue[i] < 22)
        {
            sideline_zero_flag = false; ////sideline_zero_flag預設為true
            break;
        }
    }
    for (int i = 0; i < strategy_info->color_mask_subject_cnts[1]; i++)
    {
        cntTopYellow_x = 0;
        cntBottomYellow_x = 0;
        ROS_INFO("Area = %d",strategy_info->color_mask_subject[1][i].size);
        if ( strategy_info->color_mask_subject[1][i].size > 35 && strategy_info->color_mask_subject[1][i].YMax > 230)
        {
            BottomYellowPoint = (strategy_info->color_mask_subject[1][i].YMax) * 320;
            TopYellowPoint = (strategy_info->color_mask_subject[1][i].YMin) * 320;
            for (int j = 0; j < 320; j++)
            {
                if (strategy_info->label_model[TopYellowPoint + j] != (int)LabelMark::YellowLabel)
                {
                    cntTopYellow_x++;
                }
                else
                {
                    break;
                }
            }
            for (int j = 0; j < 320; j++)
            {
                if (strategy_info->label_model[BottomYellowPoint + j] != (int)LabelMark::YellowLabel)
                {
                    cntBottomYellow_x++;
                }
                else
                {
                    break;
                }
            }
            printinfo();
            sidelineslope = (float)(TopYellowPoint - BottomYellowPoint) / (float)(cntBottomYellow_x - cntTopYellow_x);
            tool->Delay(1000);

            if (sidelineslope > 0)
            {
                leftsidelinewarning = true;
                rightsidelinewarning = false;
            }
            else if (sidelineslope < 0)
            {
                rightsidelinewarning = true;
                leftsidelinewarning = false;
            }

            printinfo();
            if (m_state != P_FM_TURNHEAD)
            {
                if (sideline_zero_flag == false)
                {
                    if (leftsidelinewarning == true)
                    {
                        walking_state = continuousValue_Ry;
                        break;
                    }
                    else if (rightsidelinewarning == true)
                    {
                        walking_state = continuousValue_Ly;
                        ROS_INFO("Ly3");
                        break;
                    }
                }
                m_state = P_MATRIX_CALCULATE;
                break;
            }
        }
        else
        {
            rightsidelinewarning = false;
            leftsidelinewarning = false;
        }
    }
    printinfo();
}
void KidsizeStrategy::printinfo()
{
    ROS_INFO("\n\n\n\n\n");
    //ROS_INFO("%s", m_state_string.c_str());
    if (zero_flag)
    {
        //walking_state_string="forward";
        ROS_INFO("zero_flag = true");
    }
    else
        ROS_INFO("zero_flag = false");
	
	ROS_INFO("continous_angle_offest = %d",continous_angle_offest);

    ROS_INFO("%s",walking_state_string.c_str());
    ROS_INFO("continuousValue_x = %5d", continuousValue_x);
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
    ROS_INFO("true_RMoveValue = %d", true_RMoveValue);
    ROS_INFO("true_LMoveValue = %d", true_LMoveValue);
    ROS_INFO("/////////////Red Door/////////////");
    ROS_INFO("Red Door slope_avg = %f", slope_avg);
    if (Center_door)
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
        ROS_INFO("first_act_flag = false");
    ROS_INFO("side line slope = %f", sidelineslope);
    //ROS_INFO("\n\n\n");
}
void KidsizeStrategy::turnslope()
{

    ///ROS_INFO("111");
    /*while(continuousValue_x != dirdata[30])
    {
        continuousValue_x -= 100;
        ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32],IMU_continuous);
        tool->Delay(100);
    }*/
    if (continuousValue_x > dirdata[30])
    {
        ///ROS_INFO("X > dirdata[30]");
        while (continuousValue_x > dirdata[30])
        {
            continuousValue_x -= 50;
            ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32], IMU_continuous);
            tool->Delay(50);
        }
    }
    else
    {
        ///ROS_INFO("X < dirdata[30]");
        while (continuousValue_x < dirdata[30])
        {
            continuousValue_x += 50;
            ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32], IMU_continuous);
            tool->Delay(100);
        }
    }

    tool->Delay(500);
    ///ROS_INFO("222");

    bool Check_label_model_flag = true;
    rest_flag = false;
    Continuous_flag = true;
    int Check_Xmin = 160;
    int Check_Xmax = 160;
    int Check_Ymax = 0;
    int Check_Ymax_another_side = 0;
    slope = 1000000.0;
    ///ROS_INFO("turnslope");
    turnslope_flag = true;
    while (turnslope_flag == true)
    {
        strategy_info->get_image_flag = true;
        ros::spinOnce();
        for (int i = 0; i <= strategy_info->color_mask_subject_cnts[2]; i++)
        {
            if (strategy_info->color_mask_subject[2][i].size > 6000)
            {
                if (m_obs_vector.size() > 1)
                {
                    ///ROS_INFO("1");
                    for (int k = 0; k < m_obs_vector.size() - 1; k++)
                    {
                        int y_min = m_obs_vector[k + 1].y_min;
                        int y_max = m_obs_vector[k].y_max;
                        if (y_min > y_max)
                        {
                            m_obs_vector[k].y_max = m_obs_vector[k].y_min;
                            m_finish_obs_vector.push_back(obs_data);
                        }
                    }
                }
                for (int h = strategy_info->color_mask_subject[2][i].YMax; h >= strategy_info->color_mask_subject[2][i].YMin; h--)
                {
                    for (int w = strategy_info->color_mask_subject[2][i].XMin; w <= strategy_info->color_mask_subject[2][i].XMax; w++)
                    {
                        if (strategy_info->label_model[320 * h + w] == 0x04)
                        {
                            for (int j = 1; j <= 3; j++)
                            {
                                if (strategy_info->label_model[320 * h + w + j] != 0x04)
                                {
                                    /////ROS_INFO("2");
                                    Check_label_model_flag = false;
                                    break;
                                }
                            }
                            if (Check_label_model_flag == true)
                            {
                                /////ROS_INFO("6");
                                if (h > Check_Ymax)
                                {
                                    if (w <= 160)
                                    {
                                        ///ROS_INFO("3");
                                        red_obs_left_coordinates.x = w;
                                        red_obs_left_coordinates.y = h;
                                        turn_direction = Right;
                                    }
                                    else
                                    {
                                        ///ROS_INFO("4");
                                        red_obs_right_coordinates.x = w;
                                        red_obs_right_coordinates.y = h;
                                        turn_direction = Left;
                                    }
                                    Check_Ymax = h;
                                }
                                switch (turn_direction)
                                {
                                case Right:
                                    if (w >= Check_Xmax)
                                    {
                                        if (h >= Check_Ymax_another_side)
                                        {
                                            red_obs_right_coordinates.x = w;
                                            red_obs_right_coordinates.y = h;
                                            Check_Ymax_another_side = h;
                                            Check_Xmax = w;
                                        }
                                    }
                                    break;
                                case Left:
                                    if (w <= Check_Xmin)
                                    {
                                        if (h >= Check_Ymax_another_side)
                                        {
                                            red_obs_left_coordinates.x = w;
                                            red_obs_left_coordinates.y = h;
                                            Check_Xmin = w;
                                            Check_Ymax_another_side = h;
                                        }
                                    }
                                    break;
                                }
                            }
                            Check_label_model_flag = true;
                        }
                    }
                }
            }
        }
        ///ROS_INFO("final_right_x = %f , final_right_y = %f",red_obs_right_coordinates.x,red_obs_right_coordinates.y);
        ///ROS_INFO("final_left_x = %f , final_left_y = %f",red_obs_left_coordinates.x,red_obs_left_coordinates.y);
        ///ROS_INFO("5");
        slope = (red_obs_right_coordinates.y - red_obs_left_coordinates.y) / (red_obs_right_coordinates.x - red_obs_left_coordinates.x);
        ///ROS_INFO("6");
        if (slope < 0)
        {
            if (abs(slope) > 0.3)
            { ///ROS_INFO("7");
                if (Continuous_flag)
                {
                    /*///ROS_INFO("xxxxxxxxxx");
              while(continuousValue_x != 0)
                {
                continuousValue_x -= 100;
                tool->Delay(100);
                ///ROS_INFO("ffffff");
                ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32]);
                }*/
                    ///ROS_INFO("8");
                    ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_single);
                    Continuous_flag = false;
                    ///ROS_INFO("9");
                    tool->Delay(3000);
                    ///ROS_INFO("10");
                }
                ///ROS_INFO("%f",slope);
                ///ROS_INFO("WC_BIGLEFT");
                ///ROS_INFO("11");
                ros_com->sendBodyAuto(dirdata[12], dirdata[13], 0, dirdata[14], WalkingMode::Single_third, IMU_single);
                tool->Delay(2000);
                slope_flag = true;
                ///ROS_INFO("12");
                //ros_com->sendBodyAuto(0, 0, 0, 0,WalkingMode::ContinuousStep);

                //Continuous_flag=true;
            }
        }
        else
        {
            if (abs(slope) > 0.3)
            {
                if (Continuous_flag)
                {
                    /*while(continuousValue_x != 0)
                {
                continuousValue_x -= 100;
                tool->Delay(100);
                ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32]);
                }*/
                    ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_single);
                    Continuous_flag = false;
                    ///ROS_INFO("13");
                    tool->Delay(3000);
                    ///ROS_INFO("14");
                }
                ///ROS_INFO("%f",slope);
                ///ROS_INFO("WC_BIGRIGHT");
                ///ROS_INFO("15");
                ros_com->sendBodyAuto(dirdata[21], dirdata[22], 0, dirdata[23], WalkingMode::Single_third, IMU_single);
                tool->Delay(2000);
                slope_flag = true;
                ///ROS_INFO("16");
                //ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep);

                //Continuous_flag=true;
            }
        }
        if (abs(slope) < 0.3)
        {
            ///ROS_INFO("abs(slope)<0.3");
            turnslope_flag = false;
            if (slope_flag == true)
            {
                ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_single);
                slope_flag = false;
            }
            ///ROS_INFO("leave turnslope");
            tool->Delay(1000);
        }
    }
}

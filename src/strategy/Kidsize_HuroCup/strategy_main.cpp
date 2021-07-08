#include "strategy/strategy_main.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OBSstrategy");
    ros::NodeHandle nh;
    KidsizeStrategy KidsizeStrategy(nh);

    ros::Rate loop_rate(20);

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
        //ROS_INFO("Speed_Max = %d",Speed_Max);

        Parameter();
        if (!Continuous_flag) //起步步態
        {
            //ROS_INFO("senbdbodyauto");  
            //ros_com->sendBodySector(4); //動作磁區
            //tool->Delay(1000);
            //ros_com->sendBodyAuto(continuous_stay_X, continuous_stay_Y, 0, continuous_stay_T, WalkingMode::ContinuousStep, IMU_continuous); //ros_com->sendBodyAuto(-450, 0, 0,-3, WalkingMode::ContinuousStep,IMU_continuous);
            //tool->Delay(500);
            Continuous_flag = true;
        }
        //ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1603, 300);
        //tool->Delay(100);
        //ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300); //頭部馬達刻度（左右）左正右負
        //tool->Delay(100);

        
                

        if(Dy < Dy_Max)
        {
            if(continuousX_Speed > continuous_stay_X)    //直走轉原地
            {
                //ROS_INFO("continuous_forword_X > Speed_Max");
                while(continuousX_Speed > continuous_stay_X)
                {
                    continuousX_Speed -= 50;
                    ros_com->sendContinuousValue(continuousX_Speed, continuous_stay_Y, 0, continuous_stay_T,IMU_continuous);
                    tool->Delay(50);
                    //ROS_INFO(" %d",continuousX_Speed);
                    strategy_info->get_image_flag = true;   
                    ros::spinOnce();
                }
            }

            /*if (strategy_info->color_mask_subject_cnts[1] || strategy_info->color_mask_subject_cnts[2] != 0)    //BY障礙物
            {
                if(Dx > 0)
                {
                    while (Dx > 0)
                    {
                        
                    }
                    
                }
                else
                {

                }
            }
            else if(strategy_info->color_mask_subject_cnts[5] != 0)                                             //R障礙物
            {
                for(int i = 0;i < strategy_info->color_mask_subject_cnts[5]; i++)
                {
                    if(strategy_info->color_mask_subject[5][i].size > R_size)
                    {
                        RedDoorCase();
                    }
                }
            }*/
        }
        else
        {
            if(continuousX_Speed < Speed_Max)           //原地轉直走
            {
                //ROS_INFO("continuous_forword_X < Speed_Max");
                while(continuousX_Speed < Speed_Max)
                {
                    continuousX_Speed += 50;
                    ros_com->sendContinuousValue(continuousX_Speed, continuous_stay_Y, 0, continuous_stay_T,IMU_continuous);
                    tool->Delay(50);
                    //ROS_INFO(" %d",continuousX_Speed);
                    strategy_info->get_image_flag = true;   
                    ros::spinOnce();
                }
            }
        }
    }
        
    else //策略指撥關閉
    {
        if (Continuous_flag)
        {
            ROS_INFO("stopppppp");
            //ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous); //關閉連續步態
            Continuous_flag = false;
            //tool->Delay(1500);
        }
        //ros_com->sendBodySector(5);
        //tool->Delay(1000);
        //ros_com->sendBodySector(29);
        //tool->Delay(1000);
        
        Parameter();
    }
}

/*
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
void KidsizeStrategy::RedDoorCase()
{

}



void KidsizeStrategy::Parameter() //步態參數之讀檔
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
        continuous_stay_X = tool->readvalue(fin, "continuous_stay_X", 0);
        continuous_stay_Y = tool->readvalue(fin, "continuous_stay_Y", 0);
        continuous_stay_T = tool->readvalue(fin, "continuous_stay_T", 0);
        fin.getline(temp, sizeof(temp));
        continuous_TurnRight_X = tool->readvalue(fin, "continuous_TurnRight_X", 0);
        continuous_TurnRight_Y = tool->readvalue(fin, "continuous_TurnRight_Y", 0);
        continuous_TurnRight_T = tool->readvalue(fin, "continuous_TurnRight_T", 0);
        fin.getline(temp, sizeof(temp));
        continuous_TurnLeft_X = tool->readvalue(fin, "continuous_TurnLeft_X", 0);
        continuous_TurnLeft_Y = tool->readvalue(fin, "continuous_TurnLeft_Y", 0);
        continuous_TurnLeft_T = tool->readvalue(fin, "continuous_TurnLeft_T", 0);
        fin.getline(temp, sizeof(temp));
        Dy_Max = tool->readvalue(fin, "Dy_Max", 0);
        Speed_Max = tool->readvalue(fin, "Speed_Max", 0);
        BY_size = tool->readvalue(fin, "BY_size", 0);
        R_size = tool->readvalue(fin, "R_size", 0);
        fin.close();
    }
    catch (exception e)
    {
    }
}
void KidsizeStrategy::GetDeepMatrix(const strategy::DeepMatrix &msg) //深度矩陣之副函式   (GetParameter)
{
    for (int i = 0; i < 32; i++)
    {
        //DeepMatrixValue[i] = msg.DeepMatrix[i];
        Dx = msg.Dx;
        Dy = msg.Dy;
    }
}


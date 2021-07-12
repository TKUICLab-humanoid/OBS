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

    if (strategy_info->getStrategyStart()) //strategy start
    {
        readparameter();

        switch(strategy_state)
        {
            case INIT:
                ROS_INFO("state = INIT");
                //initial parameter//
                continuous_angle_offset = 0;

                //preturn
                if(preturn_enable)
                    if(preturn_dir = 1) // turn left
                    {
                        ROS_INFO("preturn left");
                        ros_com->sendContinuousValue(preturn_speed, 0, 0,preturn_theta, IMU_continuous);
                        tool->Delay(preturn_time);
                    }
                    else if(preturn_dir = 2) // turn right
                    {
                        ROS_INFO("preturn right");
                        ros_com->sendContinuousValue(preturn_speed, 0, 0, -preturn_theta, IMU_continuous);
                        tool->Delay(preturn_time);
                    }
                else
                    ROS_INFO("No preturn");

            strategy_state = AVOID;   
            break;

            case AVOID:
            ROS_INFO("state = AVOID");

            break;

            default :
                ROS_INFO("default");
            break;
        }
    }
    else
    {
        strategy_state = INIT;
    }

}

void KidsizeStrategy::printinfo()
{
                    
    ROS_INFO("nearest_distance_y = %d",nearest_distance_y);
    ROS_INFO("x_boundary = %.3lf",x_boundary);
    ROS_INFO("[0] = %5d,[1] = %5d,[2] = %5d",dirdata[0],dirdata[1],dirdata[2]);
    ROS_INFO("[3] = %5d,[4] = %5d,[5] = %5d",dirdata[3],dirdata[4],dirdata[5]);
    ROS_INFO("[6] = %5d,[7] = %5d,[8] = %5d",dirdata[6],dirdata[7],dirdata[8]);

    ROS_INFO("\n");
}

void KidsizeStrategy::initparameterpath()
{
    while (parameter_path == "N")
    {
        parameter_path = tool->getPackagePath("strategy");
    }
    printf("parameter_path is %s\n", parameter_path.c_str());
}

void KidsizeStrategy::GetDeepMatrix(const strategy::DeepMatrix &msg)      //void KidsizeStrategy::GetParameter(const strategy::GetParameter &msg) 
{
	nearest_distance_y = msg.Dy;

	x_boundary = msg.Dx;
    RD = msg.RD;
    LD = msg.LD;
    slope_avg = msg.slope_avg;
    LeftblueOBS_XMax = msg.LeftblueOBS_XMax;
    RightblueOBS_XMin = msg.RightblueOBS_XMin;

}

void KidsizeStrategy::load_preturn_txt() //first_move讀檔之副函式
{
    fstream fin;
    string sTmp;
    char line[100];
    char path[200];
    strcpy(path, parameter_path.c_str());
    strcat(path, "/preturn.ini");
    fin.open(path, ios::in);
    try
    {
        preturn_enable = tool->readvalue(fin, "preturn_enable", 1);
        preturn_speed = tool->readvalue(fin, "preturn_speed", 1);
        preturn_dir = tool->readvalue(fin, "preturn_dir", 1);
        preturn_theta = tool->readvalue(fin, "preturn_theta", 1);
        preturn_time = tool->readvalue(fin, "preturn_time", 1);
        fin.close();
    }
    catch (exception e)
    {
    }
}

void KidsizeStrategy::readparameter() //步態參數之讀檔   void KidsizeStrategy::walkingparameter()
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
        dirdata[0] = tool->readvalue(fin, "continuous_x_offset", 0);
        dirdata[1] = tool->readvalue(fin, "continuous_y_offset", 0);
        dirdata[2] = tool->readvalue(fin, "continuous_theta_offset", 0);
        dirdata[3] = tool->readvalue(fin, "continuous_x_offset_RIGHT", 0);
        dirdata[4] = tool->readvalue(fin, "continuous_y_offset_RIGHT", 0);
        dirdata[5] = tool->readvalue(fin, "continuous_theta_offset_RIGHT", 0);
        dirdata[6] = tool->readvalue(fin, "continuous_x_offset_LEFT", 0);
        dirdata[7] = tool->readvalue(fin, "continuous_y_offset_LEFT", 0);
        dirdata[8] = tool->readvalue(fin, "continuous_theta_offset_LEFT", 0);
        fin.close();
    }
    catch (exception e)
    {
    }
}
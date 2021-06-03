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
        if(init_flag == true)
        {
            ROS_INFO("\n\n\n\n\n\n\ninit\n\n\n\n\n\n\n");
            //ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous); 
            ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1603, 300);     //head_motion vertical
            tool->Delay(100);
            ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300);   //head_motion horizontal
            tool->Delay(100);
            init_flag = false;
        }

        if(nearest_distance_y <= dangerous_distance)
        {
            ROS_INFO("nearest_distance_y < 10");
            if(continuousValue_x > dirdata[0])
            {
                ROS_INFO("nearest_distance_y < 10(if)");
                while(continuousValue_x > dirdata[0])
                {
                    ROS_INFO("speed down");
                    continuousValue_x -= 100;
                    turn_angle = def_turn_angle();
                    //ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + turn_angle, IMU_continuous); //連續步態的值 
                    tool->Delay(100);
                    strategy_info->get_image_flag = true;                                                                                  
                    ros::spinOnce();
                    printinfo();
                }
            }
            else
            {
                ROS_INFO("nearest_distance_y < 10(else)");
                turn_angle = def_turn_angle();
                strategy_info->get_image_flag = true;                                                                                  
                ros::spinOnce();
                printinfo();
            }

        }
        else                    //nearest_distance_y > 16
        {
            ROS_INFO("nearest_distance_y > 10");
            if(continuousValue_x < maxspeed)
            {
                ROS_INFO("nearest_distance_y > 10(if)");
                while(continuousValue_x < maxspeed)
                {
                    ROS_INFO("speed up");
                    continuousValue_x += 100;
                    //ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + turn_angle, IMU_continuous); //連續步態的值 
                    tool->Delay(100);
                    strategy_info->get_image_flag = true;                                                                                  
                    ros::spinOnce();
                    turn_angle = 0;
                    printinfo();
                }
            }
            else
            {
                ROS_INFO("nearest_distance_y > 10(else)");

                strategy_info->get_image_flag = true;                                                                                  
                ros::spinOnce();
                printinfo();
            }
        }

    }
    else
    {
        //ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous); 
    }

}

void KidsizeStrategy::printinfo()
{
                    
    ROS_INFO("nearest_distance_y = %d",nearest_distance_y);
    ROS_INFO("x_avg_to_boundary = %.3lf",x_avg_to_boundary);
    ROS_INFO("[0] = %5d,[1] = %5d,[2] = %5d",dirdata[0],dirdata[1],dirdata[2]);
    ROS_INFO("[3] = %5d,[4] = %5d,[5] = %5d",dirdata[3],dirdata[4],dirdata[5]);
    ROS_INFO("[6] = %5d,[7] = %5d,[8] = %5d",dirdata[6],dirdata[7],dirdata[8]);
    ROS_INFO("turn_angle = %d",turn_angle);
    ROS_INFO("continuousValue_x = %d",continuousValue_x);
    ROS_INFO("IMU_Value = %.5lf",strategy_info->getIMUValue().Yaw);
    ROS_INFO("\n");
}

int KidsizeStrategy::def_turn_angle()
{
    if(x_avg_to_boundary > 0)                       //Dx > 0 -> obstacle in left -> turn right
    {
        ROS_INFO("x_avg_to_boundary > 0");
        if(abs(x_avg_to_boundary) < 16 && abs(x_avg_to_boundary) > 11)
        {
            ROS_INFO("abs(x_avg_to_boundary) < 16 && abs(x_avg_to_boundary) > 11");
            continous_angle_offest = -10;
        }
        else if(abs(x_avg_to_boundary) <= 11 && abs(x_avg_to_boundary) > 16)
        {
            ROS_INFO("abs(x_avg_to_boundary) < 13 && abs(x_avg_to_boundary) > 10");
            continous_angle_offest = -8;
        }
        else if(abs(x_avg_to_boundary) <= 6 && abs(x_avg_to_boundary) > 3)
        {
            ROS_INFO("abs(x_avg_to_boundary) < 10 && abs(x_avg_to_boundary) > 7");
            continous_angle_offest = -6;
        }
        else if(abs(x_avg_to_boundary) <= 3 && abs(x_avg_to_boundary) > 1)
        {
            ROS_INFO("abs(x_avg_to_boundary) < 7 && abs(x_avg_to_boundary) > 4");
            continous_angle_offest = -4;
        }
        else
        {
            ROS_INFO("abs(x_avg_to_boundary) < 1");
            continous_angle_offest = 1;
        }
    }

    else if(x_avg_to_boundary < 0)              //Dx < 0 -> obstacle in right -> turn left
    {
        ROS_INFO("x_avg_to_boundary < 0");
        if(abs(x_avg_to_boundary) < 16 && abs(x_avg_to_boundary) > 11)
        {
            continous_angle_offest = 10;
        }
        else if(abs(x_avg_to_boundary) <= 11 && abs(x_avg_to_boundary) > 6)
        {
            continous_angle_offest = 8;
        }
        else if(abs(x_avg_to_boundary) <= 6 && abs(x_avg_to_boundary) > 3)
        {
            continous_angle_offest = 6;
        }
        else if(abs(x_avg_to_boundary) <= 3 && abs(x_avg_to_boundary) > 1)
        {
            continous_angle_offest = 4;
        }
        else
        {
            continous_angle_offest = 1;
        }   
    }

    else
    {
        //ROS_INFO("IMU");
        //fix IMU
        ROS_INFO("else");
        continous_angle_offest = 0;
    }
    return continous_angle_offest;
}

void KidsizeStrategy::initparameterpath()
{
    while (parameter_path == "N")
    {
        parameter_path = tool->getPackagePath("strategy");
    }
    printf("parameter_path is %s\n", parameter_path.c_str());
}

void KidsizeStrategy::GetDeepMatrix(const strategy::DeepMatrix &msg)
{
	nearest_distance_y = msg.Dy;

	x_avg_to_boundary = msg.Dx;

}

void KidsizeStrategy::readparameter() //步態參數之讀檔
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
        maxspeed   = tool->readvalue(fin, "max_speed",0);
        dangerous_distance   = tool->readvalue(fin, "dangerous_distance",0);
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
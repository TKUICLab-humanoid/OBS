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
        ROS_INFO("strategy start");
        switch(_state)
        {
            ROS_INFO("_state");
            case P_INIT:

                ROS_INFO("P_INIT");
                readparameter();

                ROS_INFO("\n\n\n\n\n\n\ninit\n\n\n\n\n\n\n");
                //ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous); 
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1603, 300);     //head_motion vertical
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300);   //head_motion horizontal
                tool->Delay(100);
                
                IMU_Value = strategy_info->getIMUValue().Yaw;

                _state = P_First_lawyer;
                break;

            case P_First_lawyer:

                ROS_INFO("P_First_lawyer");
                ROS_INFO("nearest_distance_y = %d",nearest_distance_y);
                ROS_INFO("dangerous_distance = %d",dangerous_distance);

                if(nearest_distance_y <= dangerous_distance)
                {
                    ROS_INFO("(if)nearest_distance_y < %d",nearest_distance_y);
                    if(continuousValue_x > minspeed)
                    {
                        ROS_INFO("nearest_distance_y < %d(if)",nearest_distance_y);
                        while(continuousValue_x > minspeed)
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
                else                    //nearest_distance_y > 10
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
                    /*else
                    {
                        ROS_INFO("nearest_distance_y > 10(else)");

                        strategy_info->get_image_flag = true;                                                                                  
                        ros::spinOnce();
                        printinfo();
                    }*/
                }
                

                /*if((IMU_Value < 0 && x_boundary == 0) ||(IMU_Value > 0 && x_boundary == 31))
                {
                    _state = P_TurnHead;
                }*/
                    
                //break;

            /*case P_TurnHead:
            {
                if(IMU_Value < 0 && x_boundary == 0)
                {
                    ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 1647, 100);     //turn head left
                    tool->Delay(100);
                    ROS_INFO("need to turn left");

                }
                else if(IMU_Value > 0 && x_boundary == 31)
                {

                }
            }
*/
            default:
                ROS_INFO("case default\n\n");
                _state = P_First_lawyer;
                break;


        }

        

    }
    else
    {
        //ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous); 
        _state = P_INIT;
    }

}

void KidsizeStrategy::printinfo()
{
                    
    ROS_INFO("nearest_distance_y = %d",nearest_distance_y);
    ROS_INFO("x_boundary = %.3lf",x_boundary);
    ROS_INFO("(stay ) x = %5d,y = %5d,theta = %5d",stay.x,stay.y,stay.theta);
    ROS_INFO("(Rmove) x = %5d,y = %5d,theta = %5d",Rmove.x,Rmove.y,Rmove.theta);
    ROS_INFO("(Lmove) x = %5d,y = %5d,theta = %5d",Lmove.x,Lmove.y,Lmove.theta);
    ROS_INFO("turn_angle = %d",turn_angle);
    ROS_INFO("continuousValue_x = %d",continuousValue_x);
    ROS_INFO("IMU_Value = %.5lf",IMU_Value);
    ROS_INFO("\n");
}

int KidsizeStrategy::def_turn_angle()
{
    if(x_boundary > 0)                       //Dx > 0 -> obstacle in left -> turn right
    {
        ROS_INFO("x_boundary > 0   turn right");
        if(abs(x_boundary) < 16 && abs(x_boundary) > 11)
        {
            //ROS_INFO("abs(x_boundary) < 16 && abs(x_boundary) > 11");
            continous_angle_offest = -10;
        }
        else if(abs(x_boundary) <= 11 && abs(x_boundary) > 16)
        {
           // ROS_INFO("abs(x_boundary) < 13 && abs(x_boundary) > 10");
            continous_angle_offest = -8;
        }
        else if(abs(x_boundary) <= 6 && abs(x_boundary) > 3)
        {
            //ROS_INFO("abs(x_boundary) < 10 && abs(x_boundary) > 7");
            continous_angle_offest = -6;
        }
        else if(abs(x_boundary) <= 3 && abs(x_boundary) > 1)
        {
            //ROS_INFO("abs(x_boundary) < 7 && abs(x_boundary) > 4");
            continous_angle_offest = -4;
        }
        else
        {
            //ROS_INFO("abs(x_boundary) < 1");
            continous_angle_offest = 1;
        }
    }

    else if(x_boundary < 0)              //Dx < 0 -> obstacle in right -> turn left
    {
        ROS_INFO("x_boundary < 0  turn left");
        if(abs(x_boundary) < 16 && abs(x_boundary) > 11)
        {
            continous_angle_offest = 10;
        }
        else if(abs(x_boundary) <= 11 && abs(x_boundary) > 6)
        {
            continous_angle_offest = 8;
        }
        else if(abs(x_boundary) <= 6 && abs(x_boundary) > 3)
        {
            continous_angle_offest = 6;
        }
        else if(abs(x_boundary) <= 3 && abs(x_boundary) > 1)
        {
            continous_angle_offest = 4;
        }
        else
        {
            continous_angle_offest = 1;
        }   
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

	x_boundary = msg.Dx;

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
        minspeed   = tool->readvalue(fin, "min_speed",0);
        dangerous_distance   = tool->readvalue(fin, "dangerous_distance",0);
        stay.x = tool->readvalue(fin, "continuous_x_offset", 0);
        stay.y = tool->readvalue(fin, "continuous_y_offset", 0);
        stay.theta = tool->readvalue(fin, "continuous_theta_offset", 0);
        Rmove.x = tool->readvalue(fin, "continuous_x_offset_RIGHT", 0);
        Rmove.y = tool->readvalue(fin, "continuous_y_offset_RIGHT", 0);
        Rmove.theta = tool->readvalue(fin, "continuous_theta_offset_RIGHT", 0);
        Lmove.x = tool->readvalue(fin, "continuous_x_offset_LEFT", 0);
        Lmove.y = tool->readvalue(fin, "continuous_y_offset_LEFT", 0);
        Lmove.theta = tool->readvalue(fin, "continuous_theta_offset_LEFT", 0);
        fin.close();
    }
    catch (exception e)
    {
    }
}
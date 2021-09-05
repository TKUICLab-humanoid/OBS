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
                turn_angle = 0;
                //head motor angle set 
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1603, 300);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300); 
                tool->Delay(100);
                readpreturnparameter();
                ROS_INFO("preturn_enable = %d",preturn_enable);
                //preturn
                if(preturn_enable)
                {
                    ROS_INFO("preturn_dir = %d",preturn_dir);
                    if(preturn_dir = 1) // turn left
                    {
                        ROS_INFO("preturn left");
                        ROS_INFO("preturn_theta = %d,time = %d",preturn_theta,preturn_time);
                        ros_com->sendContinuousValue(preturn_speed, 0, 0,preturn_theta, IMU_continuous);
                        tool->Delay(preturn_time);
                    }
                    else if(preturn_dir = 2) // turn right
                    {
                        ROS_INFO("preturn right");
                        ROS_INFO("preturn_theta = %d,time = %d",preturn_theta,preturn_time);
                        ros_com->sendContinuousValue(preturn_speed, 0, 0, -preturn_theta, IMU_continuous);
                        tool->Delay(preturn_time);
                    }
                }
                else
                    ROS_INFO("No preturn");

                strategy_state = AVOID;   
                break;

            case AVOID:
                ROS_INFO("state = AVOID");
                if(Dy < dangerous_distance)         //dangerous_distance = 10
                {
                    //check reddoor or not
                    if(RD != 0 && LD != 0)
                    {
                        ROS_INFO("AVOID -> Reddoor");
                        strategy_state = REDDOOR;
                        break;
                    }
                    else
                    {
                        //slow down and rotate
                        if(Dx > 5)                  // speed-- & turn right
                        {
                            ROS_INFO("Dx > 5,speed--,turn right");
                            if(continuousValue_x > minspeed) // current speed > minspeed
                            //if(continuousValue_x > stay.x) // current speed > the speed of stepping 
                            {
                                while(continuousValue_x > minspeed)
                                {
                                    continuousValue_x -= 100;
                                    turn_angle = def_turn_angle();
                                    //ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + turn_angle, IMU_continuous); //連續步態的值 
                                    ROS_INFO("continuousValue_x = %d,turn_angle = %d",continuousValue_x,turn_angle);
                                    tool->Delay(100);
                                }

                                if(Dy > 10)         //next obstacle's distance > 10, speed++
                                {
                                    ROS_INFO("Dx > 5,Dy > 10");
                                    if(continuousValue_x < maxspeed) // current speed > minspeed
                                    //if(continuousValue_x > stay.x) // current speed > the speed of stepping 
                                    {
                                        while(continuousValue_x < maxspeed)
                                        {
                                            continuousValue_x += 100;
                                            //ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + turn_angle, IMU_continuous); //連續步態的值 
                                            ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                            tool->Delay(100);
                                        }
                                    }
                                }

                                else
                                {
                                    strategy_state = TURNHEAD;
                                    ROS_INFO("AVOID->TURNHEAD");
                                    break;
                                }
                            }
                            else
                            {
                                ROS_INFO("Dx<5,turnangle");
                                turn_angle = def_turn_angle();
                                //ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + turn_angle, IMU_continuous); 
                                tool->Delay(100);
                            }
                        }

                        else if(Dx < -5)        // speed-- & turn left
                        {
                             ROS_INFO("Dx < -5,speed--,turn left");
                            if(continuousValue_x > minspeed) // current speed > minspeed
                            //if(continuousValue_x > stay.x) // current speed > the speed of stepping 
                            {
                                while(continuousValue_x > minspeed)
                                {
                                    continuousValue_x -= 100;
                                    turn_angle = def_turn_angle();
                                    ROS_INFO("continuousValue_x = %d,turn_angle = %d",continuousValue_x,turn_angle);
                                    //ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + turn_angle, IMU_continuous);  
                                    tool->Delay(100);
                                }

                                if(Dy > 10)         //next obstacle's distance > 10, speed++
                                {
                                    ROS_INFO("Dx < -5,Dy > 10");
                                    if(continuousValue_x < maxspeed) // current speed > minspeed
                                    //if(continuousValue_x > stay.x) // current speed > the speed of stepping 
                                    {
                                        while(continuousValue_x < maxspeed)
                                        {
                                            continuousValue_x += 100;
                                            ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                            //ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + turn_angle, IMU_continuous);  
                                            tool->Delay(100);
                                        }
                                    }
                                }

                                else
                                {
                                    ROS_INFO("AVOID->TURNHEAD");
                                    strategy_state = TURNHEAD;
                                    break;
                                }
                            }
                            else
                            {
                                ROS_INFO("Dx>5,turnangle");
                                turn_angle = def_turn_angle();
                                //ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + turn_angle, IMU_continuous);  
                                tool->Delay(100);
                            }   
                        }
                    }
                }
                else
                {
                    if(continuousValue_x < maxspeed) // current speed > minspeed
                    //if(continuousValue_x > stay.x) // current speed > the speed of stepping 
                    {
                        while(continuousValue_x < maxspeed)
                        {
                            continuousValue_x += 100;
                            //ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + turn_angle, IMU_continuous);  
                            tool->Delay(100);
                        }
                    }
                }

            break;

            case TURNHEAD:
                ROS_INFO("TURNHEAD");
                if(IMU_Value < 0 && Dx > 0)         //obstacle left,need to turn right
                {
                    ROS_INFO("IMU_Value < 0 && Dx > 0");
                    ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 1447, 300);           //head turn left
                    tool->Delay(100);

                    if(Dy > 10)   //next obstacle's distance > 10
                    {
                        ROS_INFO("Dy > 10");
                        //ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + turn_angle, IMU_continuous);
                        tool->Delay(500);

                        strategy_state = AVOID;
                        break;
                    }
                    else
                    {
                        //special case 1;
                    }
                }

                else if(IMU_Value > 0 && Dx < 0)         //obstacle rigjt,need to turn left
                {
                    ROS_INFO("IMU_Value > 0 && Dx < 0");
                    ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2647, 300);           //head turn right
                    tool->Delay(100);

                    if(Dy > 10)   //next obstacle's distance > 10
                    {
                        //ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + turn_angle, IMU_continuous);
                        tool->Delay(500);

                        strategy_state = AVOID;
                        break;
                    }
                    else
                    {
                        //special case 1;
                    }
                }

                else
                {
                    strategy_state = AVOID;
                    break;
                }
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
                    
    ROS_INFO("Dy = %d",Dy);
    ROS_INFO("Dx = %.3lf",Dx);
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
    if(Dx > 0)                       //Dx > 0 -> obstacle in left -> turn right
    {
        ROS_INFO("Dx > 0   turn right");
        if(abs(Dx) < 16 && abs(Dx) > 11)
        {
            //ROS_INFO("abs(x_boundary) < 16 && abs(x_boundary) > 11");
            continuous_angle_offset = -10;
        }
        else if(abs(Dx) <= 11 && abs(Dx) > 16)
        {
           // ROS_INFO("abs(x_boundary) < 13 && abs(x_boundary) > 10");
            continuous_angle_offset = -8;
        }
        else if(abs(Dx) <= 6 && abs(Dx) > 3)
        {
            //ROS_INFO("abs(x_boundary) < 10 && abs(x_boundary) > 7");
            continuous_angle_offset = -6;
        }
        else if(abs(Dx) <= 3 && abs(Dx) > 1)
        {
            //ROS_INFO("abs(Dx) < 7 && abs(Dx) > 4");
            continuous_angle_offset = -4;
        }
        else
        {
            //ROS_INFO("abs(Dx) < 1");
            continuous_angle_offset = 1;
        }
    }

    else if(Dx < 0)              //Dx < 0 -> obstacle in right -> turn left
    {
        ROS_INFO("Dx < 0  turn left");
        if(abs(Dx) < 16 && abs(Dx) > 11)
        {
            continuous_angle_offset = 10;
        }
        else if(abs(Dx) <= 11 && abs(Dx) > 6)
        {
            continuous_angle_offset = 8;
        }
        else if(abs(Dx) <= 6 && abs(Dx) > 3)
        {
            continuous_angle_offset = 6;
        }
        else if(abs(Dx) <= 3 && abs(Dx) > 1)
        {
            continuous_angle_offset = 4;
        }
        else
        {
            continuous_angle_offset = 1;
        }   
    }

    return continuous_angle_offset;
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
	Dy = msg.Dy;

	Dx = msg.Dx;

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
        maxspeed = tool->readvalue(fin, "max_speed",0);
        minspeed = tool->readvalue(fin, "min_speed",0);
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


void KidsizeStrategy::readpreturnparameter() //pretur參數之讀檔
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
        preturn_enable = tool->readvalue(fin, "preturn_enable",1);
        preturn_speed = tool->readvalue(fin, "preturn_speed",1);
        preturn_dir = tool->readvalue(fin, "preturn_dir",1);
        preturn_theta = tool->readvalue(fin, "preturn_theta", 1);
        preturn_time = tool->readvalue(fin, "preturn_time", 1);
        fin.close();
    }
    catch (exception e)
    {
    }
}

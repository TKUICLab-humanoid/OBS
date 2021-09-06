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
        ROS_INFO("readparameter");
        switch(strategy_state)
        {
            case INIT:
                ROS_INFO("state = INIT");
                //initial parameter//
                continuous_angle_offset = 0;
                turn_angle = 0;
                if(!Continuous_flag)
                {
                    ROS_INFO("before start ");
                    ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous); 
                    ROS_INFO("after start ");
                    tool->Delay(1500);
                    Continuous_flag = true;
                }
                
                //head motor angle set 
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1500, 300);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300); 
                tool->Delay(100);
                readpreturnparameter();
                ROS_INFO("preturn_enable = %d",preturn_enable);
                //preturn
                if(preturn_enable)
                {
                    ROS_INFO("preturn_dir = %d",preturn_dir);
                    if(preturn_dir == 1) // turn left
                    {
                        ROS_INFO("preturn left");
                        ROS_INFO("preturn_theta = %d,time = %d",preturn_theta,preturn_time);
                        ros_com->sendContinuousValue(preturn_speed, 0, 0,preturn_theta, IMU_continuous);
                        tool->Delay(preturn_time);
                    }
                    else if(preturn_dir == 2) // turn right
                    {
                        ROS_INFO("preturn right");
                        ROS_INFO("preturn_theta = %d,time = %d",preturn_theta,preturn_time);
                        ros_com->sendContinuousValue(preturn_speed, 0, 0, preturn_theta, IMU_continuous);
                        tool->Delay(preturn_time);
                    }
                }

                else
                {
                    ros_com->sendContinuousValue(stay.x, stay.y, 0, stay.theta, IMU_continuous);
                    tool->Delay(500);
                    ROS_INFO("No preturn");
                }


                //0905++++++++++
                if(in_reddoor_flag == true)
                {
                    strategy_state = REDDOOR;
                    ROS_INFO("state = REDDOOR");
                }
                //0905++++++++++



                strategy_state = AVOID;  
                /*
                else
                {
                    ROS_INFO("No preturn");
                    strategy_state = AVOID; 
                } 
                */ 
                break;

            case AVOID:
                printinfo();
                ROS_INFO("state = AVOID");
                ROS_INFO("()continuousValue_x = %d",continuousValue_x);
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
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); //連續步態的值 
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
                                            ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); //連續步態的值 
                                            ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                            tool->Delay(100);
                                        }
                                    }
                                }

                                /*else
                                {
                                    strategy_state = TURNHEAD;
                                    ROS_INFO("AVOID->TURNHEAD");
                                    break;
                                }*/
                            }
                            else
                            {
                                ROS_INFO("Dx<5,turnangle");
                                turn_angle = def_turn_angle();
                                ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
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
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous);  
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
                                            ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous);  
                                            tool->Delay(100);
                                        }
                                    }
                                }

                                /*else
                                {
                                    ROS_INFO("AVOID->TURNHEAD");
                                    strategy_state = TURNHEAD;
                                    break;
                                }*/
                            }
                            else
                            {
                                ROS_INFO("Dx>5,turnangle");
                                turn_angle = def_turn_angle();
                                ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous);  
                                tool->Delay(100);
                            }   
                        }
                
                    }
                }
                else
                {
                    ROS_INFO("Dy > 10");
                    if(continuousValue_x < maxspeed) // current speed > minspeed
                    //if(continuousValue_x > stay.x) // current speed > the speed of stepping 
                    {
                        while(continuousValue_x < maxspeed)
                        {
                            continuousValue_x += 100;
                            ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta, IMU_continuous);  
                            tool->Delay(100);
                        }
                    }
                }

            break;

            /*case TURNHEAD:
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

            //0905++++++++++++++++
            case REDDOOR:
                ROS_INFO("state = REDDOOR");

                //*******
                //減速到原地踏步
                //*******
                
                if(abs(slope_avg) > 0.3)
                {
                    slope();
                }
                else
                {
                    if(RD < LD)             //對紅門做位移
                    {
                        ROS_INFO("LEFT_MOVE");
                        ros_com->sendContinuousValue(LeftMove_X, LeftMove_Y, 0,LeftMove_T, IMU_continuous);
                        tool->Delay(100);
                    }
                    else if(RD > LD)
                    {
                        ROS_INFO("RIGHT_MOVE");
                        ros_com->sendContinuousValue(RightMove_X, RightMove_Y, 0, RightMove_T, IMU_continuous);
                        tool->Delay(100);
                    }
                    else if(RD == LD)       //對下方藍模做比較
                    {
                        if(L_XMAX > 50)
                        {
                            ROS_INFO("RIGHT_MOVE");
                            ros_com->sendContinuousValue(RightMove_X, RightMove_Y, 0, RightMove_T, IMU_continuous);
                            tool->Delay(100);
                        }
                        else if(R_XMIN > 50)
                        {
                            ROS_INFO("LEFT_MOVE");
                            ros_com->sendContinuousValue(LeftMove_X, LeftMove_Y, 0,LeftMove_T, IMU_continuous);
                            tool->Delay(100);
                        }
                        else if(L_XMAX < 50 || R_XMIN < 50 || LeftblueOBS_XMax < 50 && RightblueOBS_XMin > 260)
                        {
                            strategy_state = CRAWL;
                        }
                    }
                }
                tool->Delay(100);
            

                strategy_state = CRAWL;
            break;

            case CRAWL:
                ROS_INFO("state = CRAWL");

                if(abs(slope_avg) > 0.3)    //確認斜率
                {
                    slope();
                }
                else                        //爬行
                {
                    //ros_com->sendHeadMotor(HeadMotorID::VerticalID, 2500, 600);
                    //tool->Delay(100);
                    //ros_com->sendBodySector(5);
                    //tool->Delay(1000);
                    //ros_com->sendBodySector(6);
                    //tool->Delay(3000);

                    for (int crwtime = 0; crwtime <= 20; crwtime++)
                    {
                        ROS_INFO("crw");
                        //strategy_info->get_image_flag = true;
                        //ros::spinOnce();
                        for (int i = 0; i < strategy_info->color_mask_subject_cnts[2]; i++)
                        {
                            if (strategy_info->color_mask_subject[2][i].size > 32000)
                            {
                                ROS_INFO("stand up1");
                                break;
                            }
                        }
                        strategy_info->get_image_flag = true;
                        ros::spinOnce();
                        for (int i = 0; i < strategy_info->color_mask_subject_cnts[1]; i++)
                        {
                            if (strategy_info->color_mask_subject[1][i].size > 35000)
                            {
                                ROS_INFO("stand up2");
                                break;
                            }
                        }
                        ros_com->sendBodySector(7);
                        tool->Delay(2200);

                    }
                }
                tool->Delay(100);


                strategy_state = INIT;              //INIT or AVOID???
            break;
            //0905++++++++++++++++*/

            default :
                ROS_INFO("default");
            break;
        }
    }
    else
    {
        if(Continuous_flag)
        {
            ROS_INFO("before end ");
            ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous); 
            ROS_INFO("after end");
            tool->Delay(500);
            Continuous_flag = false;
        }

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
    ROS_INFO("L_XMAX = %3d",L_XMAX);
    ROS_INFO("R_XMIN = %3d",R_XMIN);
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
            continuous_angle_offset = -13;
        }
        else if(abs(Dx) <= 11 && abs(Dx) > 16)
        {
           // ROS_INFO("abs(x_boundary) < 13 && abs(x_boundary) > 10");
            continuous_angle_offset = -10;
        }
        else if(abs(Dx) <= 6 && abs(Dx) > 3)
        {
            //ROS_INFO("abs(x_boundary) < 10 && abs(x_boundary) > 7");
            continuous_angle_offset = -7;
        }
        else if(abs(Dx) <= 3 && abs(Dx) > 1)
        {
            //ROS_INFO("abs(Dx) < 7 && abs(Dx) > 4");
            continuous_angle_offset = -3;
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
            continuous_angle_offset = 13;
        }
        else if(abs(Dx) <= 11 && abs(Dx) > 6)
        {
            continuous_angle_offset = 10;
        }
        else if(abs(Dx) <= 6 && abs(Dx) > 3)
        {
            continuous_angle_offset = 7;
        }
        else if(abs(Dx) <= 3 && abs(Dx) > 1)
        {
            continuous_angle_offset = 3;
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

void KidsizeStrategy::GetParameter(const strategy::GetParameter &msg)
{
	Dy = msg.Dy;
    Dx = msg.Dx;
    RD = msg.RD;
    LD = msg.LD;
    slope_avg = msg.slope_avg;
    LeftblueOBS_XMax = msg.LeftblueOBS_XMax;
    RightblueOBS_XMin = msg.RightblueOBS_XMin;
    in_reddoor_flag = msg.in_reddoor_flag;
    L_XMAX = msg.L_XMAX;
    R_XMIN = msg.R_XMIN;

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
        //0905++++++++
        fin.getline(temp, sizeof(temp));
        LeftMove_X = tool->readvalue(fin, "LeftMove_X", 0);
        LeftMove_Y = tool->readvalue(fin, "LeftMove_Y", 0);
        LeftMove_T = tool->readvalue(fin, "LeftMove_T", 0);
        RightMove_X = tool->readvalue(fin, "RightMove_X", 0);
        RightMove_Y = tool->readvalue(fin, "RightMove_Y", 0);
        RightMove_T = tool->readvalue(fin, "RightMove_T", 0);
        LeftSlope_X = tool->readvalue(fin, "LeftSlope_X", 0);
        LeftSlope_Y = tool->readvalue(fin, "LeftSlope_Y", 0);
        LeftSlope_T = tool->readvalue(fin, "LeftSlope_T", 0);
        RightSlope_X = tool->readvalue(fin, "RightSlope_X", 0);
        RightSlope_Y = tool->readvalue(fin, "RightSlope_Y", 0);
        RightSlope_T = tool->readvalue(fin, "RightSlope_T", 0);
        //0905++++++++
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

//0905++++++++
void KidsizeStrategy::slope() //正對障礙物修正之副函式
{
    if (slope_avg < 0)
    {
        if (abs(slope_avg) > 0.4 && abs(slope_avg) <= 0.5)
        {
            angle_offest = 3;
            ros_com->sendContinuousValue(LeftSlope_X, LeftSlope_Y, 0,LeftSlope_T + angle_offest, IMU_continuous);
            tool->Delay(100);
        }
        else if (abs(slope_avg) > 0.3 && abs(slope_avg) <= 0.4)
        {
            angle_offest = 2;
            ros_com->sendContinuousValue(LeftSlope_X, LeftSlope_Y, 0,LeftSlope_T + angle_offest, IMU_continuous);
            tool->Delay(100);
        }
        else
        {
            angle_offest = 0;
        }
    }
    else
    {
        if (abs(slope_avg) > 0.4 && abs(slope_avg) <= 0.5)
        {
            angle_offest = -3;
            ros_com->sendContinuousValue(RightSlope_X, RightSlope_Y, 0,RightSlope_T + angle_offest, IMU_continuous);
            tool->Delay(100);
        }
        else if (abs(slope_avg) > 0.3 && abs(slope_avg) <= 0.4)
        {
            angle_offest = -2;
            ros_com->sendContinuousValue(RightSlope_X, RightSlope_Y, 0,RightSlope_T + angle_offest, IMU_continuous);
            tool->Delay(100);
        }
        else
        {
            angle_offest = 0;
        }
    }
}
//0905++++++++
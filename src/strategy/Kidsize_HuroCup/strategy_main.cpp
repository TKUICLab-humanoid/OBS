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
                    tool->Delay(50);
                    Continuous_flag = true;
                }
                
                //head motor angle set 
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1420, 300);
                tool->Delay(50);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300); 
                tool->Delay(50);
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
                    tool->Delay(50);
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
                //printinfo();
                ROS_INFO("state = AVOID");
                ROS_INFO("(AVOID)continuousValue_x = %d",continuousValue_x);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300);           //head turn mid
                tool->Delay(50);                           
                if((Dy <= b_dangerous_distance && b_obs_flag == true) || (Dy <= y_dangerous_distance && y_obs_flag == true))         
                {
                    if(RD != 0 && LD != 0)
                    {
                        ROS_INFO("AVOID -> Reddoor");
                        strategy_state = REDDOOR;
                        break;
                    }
                    else
                    {
                        //slow down 

                        if(14 >= Dx && Dx >= 2)                  // speed-- 
                        {
                            //ROS_INFO("Dx > 1,speed--,turn right");
                            //ROS_INFO("turn r continuousValue_x = %d",continuousValue_x);
                            //if(continuousValue_x >= midspeed) // current speed > minspeed
                            if(continuousValue_x > stay.x) // current speed > the speed of stepping 
                            {
                                while(continuousValue_x > stay.x)
                                {
                                    ROS_INFO("Dx > 1,speed--,turn right");
                                    continuousValue_x -= 100;
                                    //turn_angle = def_turn_angle();
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); //連續步態的值 
                                    ros::spinOnce();
                                    tool->Delay(10);

                                    // if(continuousValue_x % 300 == 0)
                                    // {
                                    //     ros::spinOnce();
                                    //     tool->Delay(60);
                                    // }
                                    
                                    /*if(Dx >=  10)
                                    {
                                        if(abs(IMU_Value) > 8 && abs(IMU_Value) < 55)
                                        {5
                                            strategy_state = TURNHEAD;
                                            break;
                                                
                                            

                                        }
                                    }*/
                                }   
                            }
                            else if( Dx >= 6 &&  abs(IMU_Value) > 20 ) 
                            {
                                ROS_INFO("imu befor turn right");
                                IMU_Value = get_IMU();
                                IMU_theta = IMU_Modify();
                                ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                ROS_INFO("IMU_Value = %lf",IMU_Value);
                                ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                ros::spinOnce();
                                tool->Delay(70);
                            }
                            else  //turn right
                            {
                                ROS_INFO("turn right & x_value = step speed");
                                turn_angle = def_turn_angle();
                                ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                ROS_INFO("turn_angle = %d",turn_angle);
                                ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                if(abs(IMU_Value) > 80)
                                {
                                    ros_com->sendContinuousValue(Lmove.x, Lmove.y, 0, Lmove.theta - turn_angle, IMU_continuous);
                                }
                                else
                                {
                                    ros_com->sendContinuousValue(Rmove.x, Rmove.y, 0, Rmove.theta + turn_angle, IMU_continuous); 
                                }
                                //ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous);
                                ros::spinOnce();
                                tool->Delay(50);
                               /* ROS_INFO("Dx < 1,turnangle");
                                turn_angle = def_turn_angle();
                                ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
                                tool->Delay(100);
                                ROS_INFO("dangerous_distance_else");
                                turnhead_flag = true;
                                strategy_state = TURNHEAD;*/

                            }
                        }
                        
                        else if(-14 <= Dx && Dx <= -2)        // speed-- 
                        {
                            //ROS_INFO("Dx < -1,speed--,turn left");
                            //ROS_INFO("turn l continuousValue_x = %d",continuousValue_x);
                            //if(continuousValue_x >= midspeed) // current speed > minspeed
                            if(continuousValue_x > stay.x) // current speed > the speed of stepping 
                            {
                                while(continuousValue_x > stay.x)
                                {
                                    ROS_INFO("Dx < -1,speed--,turn left");
                                    continuousValue_x -= 100;
                                    //turn_angle = def_turn_angle();
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous);  
                                    ros::spinOnce();
                                    tool->Delay(10);

                                    // if(continuousValue_x % 300 == 0)
                                    // {
                                    //     ros::spinOnce();
                                    //     tool->Delay(50);
                                    // }
                                    /*if(Dx <=  -10)
                                    {
                                        if(abs(IMU_Value) > 8)
                                        {
                                            ROS_INFO("2Dx == 0 || Dx == -31");
                                            /*if(Dy > 20)         //next obstacle's distance > 10, speed++
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

                                                else
                                                {
                                                    ROS_INFO("AVOID->TURNHEAD");
                                                    turnhead_flag = true;
                                                    strategy_state = TURNHEAD;
                                                    break;
                                                //}
                                            //}

                                        }
                                    }*/
                                }
                                
                                /*if(continuousValue_x == midspeed)
                                {
                                    //if(abs(IMU_Value) > 0)
                                    {
                                        
                                        ROS_INFO("AVOID->TURNHEAD");
                                        strategy_state = TURNHEAD;
                                        break;
                                          
                                    }                               
                                }*/
                            }
                            else if( Dx <= -6 &&  abs(IMU_Value) > 20 ) 
                            {
                                ROS_INFO("imu befor turn left");
                                IMU_Value = get_IMU();
                                IMU_theta = IMU_Modify();
                                ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                ROS_INFO("IMU_Value = %lf",IMU_Value);
                                ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                ros::spinOnce();
                                tool->Delay(70);
                                
                            } 
                            else  // turn left
                            {
                                ROS_INFO("turn left & step speed");
                                turn_angle = def_turn_angle();
                                ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                ROS_INFO("turn_angle = %d",turn_angle);
                                ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                if(abs(IMU_Value) > 80)
                                {
                                    ros_com->sendContinuousValue(Rmove.x, Rmove.y, 0, Rmove.theta - turn_angle, IMU_continuous);
                                }
                                else
                                {
                                    ros_com->sendContinuousValue(Lmove.x, Lmove.y, 0, Lmove.theta + turn_angle, IMU_continuous); 
                                }  
                                ros::spinOnce();
                                //ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous);  
                                tool->Delay(50);
                            }  
                        }
                        /*else if(14 < abs(Dx) < 17)
                        {
                            ROS_INFO("2 > Dx > -2 || Dx == 0 || Dx == -31");
                        }
                        )*/
                        
                        else if( ( (Dx < 17 && Dx > 14) ||(Dx < -14 && Dx > -17) ) && l_center_Dy < 20 && r_center_Dy < 20 ) //l_center_Dy < 20 && r_center_Dy < 20
                        {
                            ROS_INFO("Dx = %5f",Dx);
                            ROS_INFO("l_center_Dy = %d, r_center_Dy = %d",l_center_Dy,r_center_Dy);
                            ROS_INFO("ready to turnhead");
                            if(continuousValue_x > stay.x) // current speed > the speed of stepping 
                            {
                                while(continuousValue_x > stay.x)
                                {
                                    ROS_INFO("speed-- in turnhead");
                                    continuousValue_x -= 100;
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); //連續步態的值 
                                    ros::spinOnce();
                                    tool->Delay(10);
                                }   
                            }
                            else if( abs(IMU_Value) > 10 )//imu fix before turnhead
                            {

                                ROS_INFO("imu fix before turnhead");
                                IMU_Value = get_IMU();
                                IMU_theta = IMU_Modify();
                                ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                ROS_INFO("IMU_Value = %lf",IMU_Value);
                                ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                ros::spinOnce();
                                tool->Delay(70);
                            }
                            else//ready to turnhead
                            {
                                ROS_INFO("AVOID--->TURNHEAD");
                                turnhead_flag = true;
                                strategy_state = TURNHEAD;
                            }
                        }
                        else //2 > Dx > -2 || Dx == 0 || Dx == -31
                        {
                            ROS_INFO("2 > Dx > -2 || Dx == 0 || Dx == -31");
                            ROS_INFO("Dx = %5f",Dx);
                            if(continuousValue_x < maxspeed ) // speed up
                            {
                                while(continuousValue_x < maxspeed)
                                {
                                    //ROS_INFO("NO TURNHEAD");
                                    continuousValue_x += 100;
                                    ROS_INFO("void speed up");
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta , IMU_continuous);  
                                    ros::spinOnce();
                                    tool->Delay(70);
                                    
                                    layer_flag = true;
                                    //ROS_INFO("layer_flag == true");
                                    break;
                                }

                            }
                            else if(continuousValue_x == maxspeed) 
                            {   
                                ROS_INFO("void speed up finish");
                                ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                ROS_INFO("turn_angle = %d",turn_angle);
                                ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta , IMU_continuous);  
                                ros::spinOnce();
                                tool->Delay(70);
                                
                            }
                        }
                
                    }
                }
                else
                {
                    ROS_INFO("Dy > %d or %d",b_dangerous_distance,y_dangerous_distance);
                    if(continuousValue_x < maxspeed) // speed up
                    //if(continuousValue_x > stay.x) // current speed > the speed of stepping 
                    {
                        while(continuousValue_x < maxspeed)
                        { 
                            continuousValue_x += 100;
                            
                            if(abs(IMU_Value) < 20)
                            {
                                ROS_INFO("no_obs speed up & use imu");
                                IMU_Value = get_IMU();
                                IMU_theta = IMU_Modify();
                                ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                ROS_INFO("IMU_Value = %lf",IMU_Value);
                                ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous);  
                                ros::spinOnce();
                                tool->Delay(70);
                                ROS_INFO("Dy = %d",Dy);
                            }
                            else 
                            {
                                ROS_INFO("no_obs speed up & use angle");
                                //continuousValue_x += 100;
                                //IMU_Value = get_IMU();
                                //IMU_theta = IMU_Modify();
                                //ROS_INFO("IMU_Value = %lf",IMU_Value);
                                ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                ROS_INFO("turn_angle = %d",turn_angle);
                                ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta , IMU_continuous); 
                                ros::spinOnce();
                                tool->Delay(70);
                                //layer_flag = true;
                                //ROS_INFO("layer_flag == true");
                                ROS_INFO("Dy = %d",Dy);
                            }

                        }

                    }
                    else if(continuousValue_x == maxspeed)
                    {
                        if(abs(IMU_Value) < 20)
                        {
                            ROS_INFO("no_obs high speed & use imu");
                            IMU_Value = get_IMU();
                            IMU_theta = IMU_Modify();
                            ROS_INFO("continuousValue_x = %d",continuousValue_x);
                            ROS_INFO("IMU_Value = %lf",IMU_Value);
                            ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                            ros::spinOnce(); 
                            tool->Delay(70);
                            ROS_INFO("Dy = %d",Dy);
                        }
                        else 
                        {
                            ROS_INFO("no_obs high speed & use angle");
                            //continuousValue_x += 100;
                            //IMU_Value = get_IMU();
                            //IMU_theta = IMU_Modify();
                            //ROS_INFO("IMU_Value = %lf",IMU_Value);
                            ROS_INFO("continuousValue_x = %d",continuousValue_x);
                            ROS_INFO("turn_angle = %d",turn_angle);
                            ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                            ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta , IMU_continuous);  
                            ros::spinOnce();
                            tool->Delay(70);
                            //layer_flag = true;
                            //ROS_INFO("layer_flag == true");
                            ROS_INFO("Dy = %d",Dy);
                        }
                            
                            
                        
                    }
                    /*if( (layer_sum - 1) > 0)
                    {
                        ROS_INFO("AVOID->TURNHEAD");
                        // turnhead_flag = true;
                        IMU_Value = get_IMU();
                        ros::spinOnce();
                        if(IMU_Value < 0)
                        {
                            LeftHead_flag = true;
                            RightHead_flag = false;
                        }
                        else if(IMU_Value > 0)
                        {
                            RightHead_flag = true;
                            LeftHead_flag = false;
                        }
                        // strategy_state = TURNHEAD;
                        break;
                    }*/
                    //ROS_INFO("Dx = %lf,",Dx);
                             
                }

            break;

            case TURNHEAD:
                ROS_INFO("TURNHEAD");
                if(turnhead_flag == true)
                {
                    ROS_INFO("turnhead_flag == true");

                    ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1520, 300);           //head up
                    ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 1447, 300);           //head turn right
                    tool->Delay(1000);           
                    ros::spinOnce();
                    tool->Delay(50);
                    ROS_INFO(" WR = %d",WR);
                    turn_WR = WR;
                    ROS_INFO(" turn_WR = %d",turn_WR);
                    tool->Delay(100);


                    ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1520, 300);
                    ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2647, 300);           //head turn left
                    tool->Delay(2000);           
                    ros::spinOnce();
                    tool->Delay(50);
                    ROS_INFO(" WL = %d",WL);
                    turn_WL = WL;
                    ROS_INFO(" turn_WL = %d",turn_WL);
                    tool->Delay(100);
                    

                    ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1420, 300); 
                    ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300);           //head turn mid
                    tool->Delay(500);
                    tool->Delay(50);
                    

                    if((turn_WL - turn_WR) > 10)// turn right(turn_WL - turn_WR) > 10
                    {
                        ROS_INFO("turn right after turnhead");
                        turnhead_flag == false;
                        ROS_INFO("turnhead_flag == false");
                        //IMU_Value = get_IMU();
                        //turn_angle = def_turn_angle();
                        if((abs(IMU_Value)) < 50 ) //obs at left (abs(IMU_Value)) < 70 
                        {
                            while((abs(IMU_Value)) < 50 )
                            {
                                IMU_Value = get_IMU();
                                ROS_INFO("IMU_Value = %lf",IMU_Value);
                                ROS_INFO("turn angle is -5");
                                ros_com->sendContinuousValue(Rmove.x, Rmove.y, 0, Rmove.theta - 10, IMU_continuous);
                                ros::spinOnce();
                                tool->Delay(50); 
                            }
                            turnhead_flag == false;
                            strategy_state = AVOID;
                        }
                        else
                        {
                            turnhead_flag == false;
                            strategy_state = AVOID;
                        }
                    }
                    else if((turn_WR - turn_WL) > 10 )//obs at right
                    {
                        ROS_INFO("turn left after turnhead");
                        turnhead_flag == false;
                        ROS_INFO("turnhead_flag == false");
                        //IMU_Value = get_IMU();
                        if((abs(IMU_Value)) < 50)// Dx >= 2 && Dx <= -2
                        {
                            while((abs(IMU_Value)) < 50 )
                            {
                                IMU_Value = get_IMU();
                                ROS_INFO("IMU_Value = %lf",IMU_Value);
                                ROS_INFO("turn angle is 5");
                                ros_com->sendContinuousValue(Lmove.x, Lmove.y, 0, Lmove.theta + 10, IMU_continuous);
                                ros::spinOnce();
                                tool->Delay(50); 
                            }
                            turnhead_flag == false;
                            strategy_state = AVOID;
                        }
                        else
                        {
                            turnhead_flag == false;
                            strategy_state = AVOID;
                        }
                    }
                    else //abs(turn_WR - turn_WL) < 10
                    {
                        ROS_INFO("abs(turn_WR - turn_WL) < 5 after turnhead");
                        //turnhead_flag == false;
                        //ROS_INFO("turnhead_flag == false");
                        turnhead_flag == false;
                        strategy_state = AVOID;
                    }

                }
                else//turnhead_flag == false
                {
                    turnhead_flag == false;
                    ROS_INFO("turnhead_flag == false");
                    strategy_state = AVOID;
                }

            break;    
            /*case TURNHEAD:
                ROS_INFO("TURNHEAD");

                if(Dy <= 12)
                {
                    ROS_INFO("Dy <= 10");

                    //if(IMU_Value < -15)         //obstacle left,need to turn right
                    
                    if(LeftHead_flag == true)         //obstacle left,need to turn right
                    {
                        
                        ROS_INFO("IMU_Value < 0 right");
                        ROS_INFO("Head Turn left");
                        ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2447, 300);           //head turn left
                        tool->Delay(100);
                        if(turnhead_flag == true)
                        {
                            if(turnhead_flag == true)  
                                ROS_INFO("turnhead_flag == true");
                            else
                                ROS_INFO("turnhead_flag == falsre");
                            
                            //IF LEFTFLAG = TRUE
                            if(continuousValue_x > minspeed) //speed down to stay
                            {
                                ROS_INFO("continuousValue_x > minspeed");
                                while(continuousValue_x > minspeed)
                                {   
                                    //ros::spinOnce();
                                    continuousValue_x -= 100;
                                    ROS_INFO("continuousValue_x > minspeed");
                                    ROS_INFO("speed --");
                                    ROS_INFO("stay.theta = %d",stay.theta);
                                    turn_angle = def_turn_angle();
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta - turn_angle = %d",stay.theta - turn_angle);
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous);
                                    //ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, 10, IMU_continuous);
                                    tool->Delay(50);
                                }
                            }
                            else if(continuousValue_x <= minspeed)
                            {
                                ROS_INFO("continuousValue_x <= minspeed");
                                while(continuousValue_x <= minspeed)
                                {
                                    ROS_INFO("continuousValue_x = minspeed");
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    turn_angle = def_turn_angle();
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous);//15
                                    tool->Delay(50);                            
                                    IMU_Value = get_IMU();
                                    ros::spinOnce();
                                    ROS_INFO("turn head IMU_Value = %lf",IMU_Value);
                                    if(IMU_Value > 35 || Dx == -31 || Dx == 0)
                                    {
                                        ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300);           //head turn left
                                        tool->Delay(100);
                                        ROS_INFO("IMU_Value > 35");
                                        turnhead_flag = false;
                                        LeftHead_flag = false;
                                        RightHead_flag = false;
                                        strategy_state = AVOID;
                                        break;
                                    }

                                    if(turnhead_flag == false)
                                    {
                                        LeftHead_flag = false;
                                        RightHead_flag = false;
                                        strategy_state = AVOID;
                                        break;
                                    }
                                }
                            }

                            else
                            {
                                ROS_INFO("continuousValue_x = minspeed");
                            } 
                            
                        }
                        else
                        {
                            if(turnhead_flag == true)  
                                ROS_INFO("turnhead_flag == true");
                            else
                                ROS_INFO("turnhead_flag == falsre");
                        }
                        
                        
                    }

                    //else if(IMU_Value > 15)         //obstacle rigjt,need to turn left
                    else if(RightHead_flag == true)         //obstacle rigjt,need to turn left
                    {
                        ROS_INFO("IMU_Value > 8");
                        ROS_INFO("Head Turn right");
                        ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 1647, 300);           //head turn right
                        tool->Delay(100);

                        if(turnhead_flag == true)
                        {
                            if(turnhead_flag == true)  
                                ROS_INFO("turnhead_flag == true");
                            else
                                ROS_INFO("turnhead_flag == falsre");

                            //if()
                            if(continuousValue_x > minspeed)
                            {
                                ROS_INFO("continuousValue_x > minspeed");
                                while(continuousValue_x > minspeed)
                                {
                                    //ros::spinOnce();
                                    ROS_INFO("continuousValue_x > minspeed");
                                    continuousValue_x -= 100;
                                    ROS_INFO("speed --");
                                    ROS_INFO("stay.theta = %d",stay.theta);
                                    turn_angle = def_turn_angle();
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta - turn_angle = %d",stay.theta - turn_angle);
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous);
                                    tool->Delay(50);
                                }
                            }
                            else if(continuousValue_x <= minspeed)
                            {
                                ROS_INFO("continuousValue_x <= minspeed");
                                while(continuousValue_x <= minspeed)
                                {
                                    ROS_INFO("continuousValue_x = minspeed");
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    turn_angle = def_turn_angle();
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous);//-15

                                    tool->Delay(50);

                                    IMU_Value = get_IMU();
                                    ros::spinOnce();
                                    ROS_INFO("turn head IMU_Value = %lf",IMU_Value);
                                    if( IMU_Value <= -35 || Dx == -31 || Dx == 0)
                                    {
                                        ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300);           //head turn mid
                                        tool->Delay(50);                            
                                        ROS_INFO("IMU_Value < -35");
                                        turnhead_flag = false;
                                        LeftHead_flag = false;
                                        RightHead_flag = false;
                                        strategy_state = AVOID;
                                        break;
                                    } 

                                }
                                if(turnhead_flag == false)
                                {
                                    LeftHead_flag = false;
                                    RightHead_flag = false;
                                    strategy_state = AVOID;
                                    break;
                                }
                            }

                           
                            else
                            {
                                ROS_INFO("continuousValue_x = minspeed");

                            }                           
                            
                        }
                        else
                        {
                        //     if(turnhead_flag == true)  
                        //         ROS_INFO("turnhead_flag == true");
                        //     else
                        //         ROS_INFO("turnhead_flag == falsre");
                        }
                    }

                    else
                    {
                        ROS_INFO("-15 < IMU < 15 ");
                        strategy_state = AVOID;
                        break;
                    }
                }
                else
                {
                    ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300);           //head turn mid
                    tool->Delay(50);
                    IMU_Value = get_IMU();
                    IMU_theta = IMU_Modify();
                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, IMU_theta, IMU_continuous);  
                    ROS_INFO("turnhead else");
                    ROS_INFO("IMU_theta = %d",IMU_theta);
                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                    tool->Delay(50);  
                    ros::spinOnce();
                    /*if(IMU_Value < 5 && IMU_Value > -5)
                    {
                        strategy_state = AVOID;
                        break;
                    }
                }

            break;*/
            

            //0905++++++++++++++++
            /*case REDDOOR:
                ROS_INFO("state = REDDOOR");

                
                //減速到原地踏步
                if(continuousValue_x > stay.x) // current speed > the speed of stepping 
                {
                    while(continuousValue_x > stay.x)
                    {
                        ROS_INFO("speed-- in REDDOOR");
                        continuousValue_x -= 100;
                        //turn_angle = def_turn_angle();
                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                        ROS_INFO("turn_angle = %d",turn_angle);
                        ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                        if(abs(IMU_Value) > 3)
                        ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous);  
                        tool->Delay(60);
                    }
                }
                else if(continuousValue_x == stay.x)
                {
                    if( abs(IMU_Value) > 1 ) 
                    {
                    ROS_INFO("imu fix in REDDOOR");
                    IMU_Value = get_IMU();
                    IMU_theta = IMU_Modify();
                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                    ROS_INFO("IMU_Value = %lf",IMU_Value);
                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                    ros::spinOnce();
                    tool->Delay(70);
                    }
                    else
                    {

                    }
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

                if( abs(IMU_Value) > 1 ) 
                {
                    ROS_INFO("imu fix in CRAWL");
                    IMU_Value = get_IMU();
                    IMU_theta = IMU_Modify();
                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                    ROS_INFO("IMU_Value = %lf",IMU_Value);
                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                    ros::spinOnce();
                    tool->Delay(70);
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
                strategy_state = AVOID;
                break;

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

float KidsizeStrategy::get_IMU()
{
    if (0 < strategy_info->getIMUValue().Yaw && strategy_info->getIMUValue().Yaw <= 180)
    {
        IMU_getValue = strategy_info->getIMUValue().Yaw;
    }
    else if (0 > strategy_info->getIMUValue().Yaw && strategy_info->getIMUValue().Yaw >= -179)
    {
        IMU_getValue = strategy_info->getIMUValue().Yaw;
    }
    return IMU_getValue;
}

int KidsizeStrategy::IMU_Modify()
{
    ROS_INFO("(IMU_Modify)IMU_Value = %lf",IMU_Value);

    if (IMU_Value < 0)
    {
        if (abs(IMU_Value) >= 90)
        {
            IMU_angle_offest = 8;
        }
        else if (abs(IMU_Value) >= 45 && abs(IMU_Value) < 90)
        {
            IMU_angle_offest = 7;
        }
        else if (abs(IMU_Value) >= 30 && abs(IMU_Value) < 45)
        {
            IMU_angle_offest = 6;
        }
        else if (abs(IMU_Value) >= 15 && abs(IMU_Value) < 30)
        {
            IMU_angle_offest = 6;
        }
        else if (abs(IMU_Value) >= 8 && abs(IMU_Value) < 15)
        {
            IMU_angle_offest = 5;
        }
        else if (abs(IMU_Value) >= 5 && abs(IMU_Value) < 8)
        {
            IMU_angle_offest = 4;
        }
        else if (abs(IMU_Value) >= 2 && abs(IMU_Value) < 5)
        {
            IMU_angle_offest = 2;
        }
        else
        {
            IMU_angle_offest = 0;
        }
    }
    else
    {
        if (abs(IMU_Value) >= 90)
        {
            IMU_angle_offest = -8;
        }
        else if (abs(IMU_Value) >= 45 && abs(IMU_Value) < 90)
        {
            IMU_angle_offest = -7;
        }
        else if (abs(IMU_Value) >= 30 && abs(IMU_Value) < 45)
        {
            IMU_angle_offest = -6;
        }
        else if (abs(IMU_Value) >= 15 && abs(IMU_Value) < 30)
        {
            IMU_angle_offest = -6;
        }
        else if (abs(IMU_Value) >= 8 && abs(IMU_Value) < 15)
        {
            IMU_angle_offest = -5;
        }
        else if (abs(IMU_Value) >= 5 && abs(IMU_Value) < 8)
        {
            IMU_angle_offest = -4;
        }
        else if (abs(IMU_Value) >= 2 && abs(IMU_Value) < 5)
        {
            IMU_angle_offest = -2;
        }
        else
        {
            IMU_angle_offest = 0;
        }
    }
    
    return IMU_angle_offest;
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
        if(abs(Dx) <= 16 && abs(Dx) > 13)
        {
            //ROS_INFO("abs(x_boundary) < 16 && abs(x_boundary) > 11");
            continuous_angle_offset = -9;
        }
        else if(abs(Dx) <= 13 && abs(Dx) > 10)
        {
           // ROS_INFO("abs(x_boundary) < 13 && abs(x_boundary) > 10");
            continuous_angle_offset = -8;
        }
        else if(abs(Dx) <= 10 && abs(Dx) > 7)
        {
           // ROS_INFO("abs(x_boundary) < 13 && abs(x_boundary) > 10");
            continuous_angle_offset = -7;
        }
        else if(abs(Dx) <= 7 && abs(Dx) > 3)
        {
            //ROS_INFO("abs(x_boundary) < 10 && abs(x_boundary) > 7");
            continuous_angle_offset = -6;
        }
        else if(abs(Dx) <= 3 && abs(Dx) > 1)
        {
            //ROS_INFO("abs(Dx) < 7 && abs(Dx) > 4");
            continuous_angle_offset = -3;
        }
        else
        {
            //ROS_INFO("abs(Dx) < 1");
            continuous_angle_offset = -1;
        }
    }

    else if(Dx < 0)              //Dx < 0 -> obstacle in right -> turn left
    {
        ROS_INFO("Dx < 0  turn left");
        if(abs(Dx) <= 16 && abs(Dx) > 13)
        {
            continuous_angle_offset = 10;
        }
        else if(abs(Dx) <= 13 && abs(Dx) > 10)
        {
            continuous_angle_offset = 8;
        }
        else if(abs(Dx) <= 10 && abs(Dx) > 7)
        {
            continuous_angle_offset = 6;
        }
        else if(abs(Dx) <= 7 && abs(Dx) > 3)
        {
            continuous_angle_offset = 5;
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
    else
    {
        
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
    WR = msg.WR;
    WL = msg.WL;
    slope_avg = msg.slope_avg;
    LeftblueOBS_XMax = msg.LeftblueOBS_XMax;
    RightblueOBS_XMin = msg.RightblueOBS_XMin;
    in_reddoor_flag = msg.in_reddoor_flag;
    b_obs_flag = msg.b_obs_flag;
    y_obs_flag = msg.y_obs_flag;
    L_XMAX = msg.L_XMAX;
    R_XMIN = msg.R_XMIN;
    l_center_Dy = msg.l_center_Dy;
    r_center_Dy = msg.r_center_Dy;

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
        midspeed = tool->readvalue(fin, "mid_speed",0);
        minspeed = tool->readvalue(fin, "min_speed",0);
        b_dangerous_distance   = tool->readvalue(fin, "b_dangerous_distance",0);
        y_dangerous_distance   = tool->readvalue(fin, "y_dangerous_distance",0);
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
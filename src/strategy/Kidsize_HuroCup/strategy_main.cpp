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

    if (strategy_info->getStrategyStart()) //策略指撥開始
    {
        readparameter();
        ROS_INFO("readparameter");
        switch(strategy_state)
        {
            case INIT://參數初始化
                ROS_INFO("state = INIT");
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
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1520, 300);
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

                strategy_state = AVOID;  //開始一般避障策略

                /*
                else
                {
                    ROS_INFO("No preturn");
                    strategy_state = AVOID; 
                } 
                */ 
                break;

            case AVOID:
                ROS_INFO("state = AVOID");
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300);           //轉頭至正中
                tool->Delay(50); 
            
                if((in_reddoor_flag == true) && (RD != 0 && LD != 0)) //符合紅門條件
                {
                    ROS_INFO("AVOID -> Reddoor");
                    strategy_state = REDDOOR;
                }
                else   //不進紅門 做藍黃避障
                {  
                    if(Dy < 24)
                    {
                        speed = def_speed();
                        //ROS_INFO("speed = %d",speed);
                        if(14 >= Dx && Dx >= 2)//障礙物在左
                        {
                            if(continuousValue_x > speed) //速度大於目前要求速度 speed-- 邊轉
                            {
                                while(continuousValue_x > speed) 
                                {
                                    if( imu_ok_flag == false &&  Dx >= 6 ) //避障前用imu對正障礙物   /*Dx >= 7 &&  abs (IMU_Value) > 20 &&*/ 
                                    {
                                        ROS_INFO("imu & speed -- befor turn right");
                                        continuousValue_x -= 100;
                                        IMU_Value = get_IMU();
                                        IMU_theta = IMU_Modify();
                                        IMU_theta = IMU_angle_offest;
                                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                        ROS_INFO("IMU_Value = %lf",IMU_Value);
                                        ROS_INFO("IMU_theta = %lf",IMU_theta);
                                        ROS_INFO("stay.theta + IMU_theta  = %lf",stay.theta + IMU_theta);
                                        ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                        tool->Delay(10);
                                        /*while( abs (IMU_Value) > 10 )   //Dx >= 7 &&  
                                        {
                                            ROS_INFO("imu befor turn right");
                                            continuousValue_x -= 100;
                                            IMU_Value = get_IMU();
                                            IMU_theta = IMU_Modify();
                                            ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                            ROS_INFO("IMU_Value = %lf",IMU_Value);
                                            ROS_INFO("IMU_theta = %lf",IMU_theta);
                                            ROS_INFO("stay.theta + IMU_theta  = %lf",stay.theta + IMU_theta);
                                            ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                            tool->Delay(10);
                                        }*/
                                    }
                                    if(abs (IMU_Value) < 5){imu_ok_flag = true;}
                                    //imu_ok_flag = true;
                                    ROS_INFO("imu_ok_flag = true");
                                    if((imu_ok_flag == true) || (imu_ok_flag == false &&  Dx < 6) ) //Dx < 7
                                    {
                                        //speed = def_speed();
                                        ROS_INFO("speed -- in turn right");
                                        continuousValue_x -= 100;
                                        turn_angle = def_turn_angle();
                                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                        ROS_INFO("turn_angle = %d",turn_angle);
                                        ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                        ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                        tool->Delay(10);
                                    }
                                }   
                            }
                            //flag = true
                            else if(continuousValue_x < speed) //速度小於目前要求速度 speed++ 邊轉
                            {
                                while(continuousValue_x < speed) 
                                {
                                    if(  imu_ok_flag == false && Dx >= 6 ) //避障前用imu對正障礙物   /*Dx >= 7 &&  abs (IMU_Value) > 20 &&*/
                                    {
                                        ROS_INFO("imu & speed ++ befor turn right");
                                        continuousValue_x += 100;
                                        IMU_Value = get_IMU();
                                        IMU_theta = IMU_Modify();
                                        IMU_theta = IMU_angle_offest;
                                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                        ROS_INFO("IMU_Value = %lf",IMU_Value);
                                        ROS_INFO("IMU_theta = %lf",IMU_theta);
                                        ROS_INFO("stay.theta + IMU_theta  = %lf",stay.theta + IMU_theta);
                                        ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                        tool->Delay(10);
                                        /*while( abs (IMU_Value) > 10 )   //Dx >= 7 &&  
                                        {
                                            ROS_INFO("imu befor turn right");
                                            continuousValue_x += 100;
                                            IMU_Value = get_IMU();
                                            IMU_theta = IMU_Modify();
                                            ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                            ROS_INFO("IMU_Value = %lf",IMU_Value);
                                            ROS_INFO("IMU_theta = %lf",IMU_theta);
                                            ROS_INFO("stay.theta + IMU_theta  = %lf",stay.theta + IMU_theta);
                                            ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                            tool->Delay(10);
                                        }*/
                                    }
                                    if(abs (IMU_Value) < 5){imu_ok_flag = true;}
                                    //imu_ok_flag = true;
                                    ROS_INFO("imu_ok_flag = true");
                                    if((imu_ok_flag == true) || (imu_ok_flag == false &&  Dx < 6) ) //Dx < 7
                                    {
                                        //speed = def_speed();
                                        ROS_INFO("speed ++ in turn right");
                                        continuousValue_x += 100;
                                        turn_angle = def_turn_angle();
                                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                        ROS_INFO("turn_angle = %d",turn_angle);
                                        ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                        ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                        tool->Delay(10);
                                    }
                                }
                            }
                            else // 等速
                            {
                                if( /*Dx >= 7 &&  abs (IMU_Value) > 20 &&*/ imu_ok_flag == false && Dx >= 6 ) //避障前用imu對正障礙物 Dx >= 7
                                {
                                    ROS_INFO("imu befor turn right");
                                    IMU_Value = get_IMU();
                                    IMU_theta = IMU_Modify();
                                    IMU_theta = IMU_angle_offest;
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("IMU_Value = %lf",IMU_Value);
                                    ROS_INFO("IMU_theta = %lf",IMU_theta);
                                    ROS_INFO("stay.theta + IMU_theta  = %lf",stay.theta + IMU_theta);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    tool->Delay(10);
                                    /*while( abs (IMU_Value) > 10 ) //Dx >= 7 &&
                                    {
                                        ROS_INFO("imu befor turn right");
                                        IMU_Value = get_IMU();
                                        IMU_theta = IMU_Modify();
                                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                        ROS_INFO("IMU_Value = %lf",IMU_Value);
                                        ROS_INFO("IMU_theta = %lf",IMU_theta);
                                        ROS_INFO("stay.theta + IMU_theta  = %lf",stay.theta + IMU_theta);
                                        ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                        tool->Delay(10);
                                    }*/
                                }
                                if(abs (IMU_Value) < 5){imu_ok_flag = true;}
                                //imu_ok_flag = true;
                                ROS_INFO("imu_ok_flag = true");
                                if((imu_ok_flag == true) || (imu_ok_flag == false &&  Dx < 6) ) //速度等於目前要求速度 保持速度旋轉  Dx < 7
                                {
                                    ROS_INFO("turn right");
                                    turn_angle = def_turn_angle();
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    tool->Delay(10);
                                }
                            }
                        }
                        else if(-14 <= Dx && Dx <= -2)//障礙物在右
                        {
                            if(continuousValue_x > speed) //速度大於目前要求速度 speed-- 邊轉
                            {
                                while(continuousValue_x > speed) //
                                {
                                    if( /*Dx <= -7 &&  abs(IMU_Value) > 10 */imu_ok_flag == false && Dx <= -6 ) //避障前用imu對正障礙物  Dx <= -7
                                    {
                                        ROS_INFO("imu & speed -- befor turn left");
                                        continuousValue_x -= 100;
                                        IMU_Value = get_IMU();
                                        IMU_theta = IMU_Modify();
                                        IMU_theta = IMU_angle_offest;
                                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                        ROS_INFO("IMU_Value = %lf",IMU_Value);
                                        ROS_INFO("IMU_theta = %lf",IMU_theta);
                                        ROS_INFO("stay.theta + IMU_theta  = %lf",stay.theta + IMU_theta);
                                        ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                        tool->Delay(10);
                                        /*while( abs(IMU_Value) > 10 )
                                        {
                                            ROS_INFO("imu befor turn left");
                                            continuousValue_x -= 100;
                                            IMU_Value = get_IMU();
                                            IMU_theta = IMU_Modify();
                                            ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                            ROS_INFO("IMU_Value = %lf",IMU_Value);
                                            ROS_INFO("IMU_theta = %lf",IMU_theta);
                                            ROS_INFO("stay.theta + IMU_theta  = %lf",stay.theta + IMU_theta);
                                            ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                            tool->Delay(10);
                                        }*/
                                    }
                                    if(abs (IMU_Value) < 5){imu_ok_flag = true;}
                                    //imu_ok_flag = true;
                                    ROS_INFO("imu_ok_flag = true");
                                    if((imu_ok_flag == true) || (imu_ok_flag == false &&   Dx > -6) ) //Dx > -7
                                    {
                                        //speed = def_speed();
                                        ROS_INFO("speed -- in turn left");
                                        continuousValue_x -= 100;
                                        turn_angle = def_turn_angle();
                                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                        ROS_INFO("turn_angle = %d",turn_angle);
                                        ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                        ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                        tool->Delay(10);
                                    }
                                }   
                            }
                            else if(continuousValue_x < speed) //速度小於目前要求速度 speed++ 邊轉
                            {
                                while(continuousValue_x < speed) 
                                {
                                    if( /*Dx <= -7 &&  abs(IMU_Value) > 10 */imu_ok_flag == false && Dx <= -6 ) //避障前用imu對正障礙物 Dx <= -7
                                    {
                                        ROS_INFO("imu & speed ++ befor turn left");
                                        continuousValue_x += 100;
                                        IMU_Value = get_IMU();
                                        IMU_theta = IMU_Modify();
                                        IMU_theta = IMU_angle_offest;
                                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                        ROS_INFO("IMU_Value = %lf",IMU_Value);
                                        ROS_INFO("IMU_theta = %lf",IMU_theta);
                                        ROS_INFO("stay.theta + IMU_theta  = %lf",stay.theta + IMU_theta);
                                        ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                        tool->Delay(10);
                                        /*while( abs(IMU_Value) > 10 )
                                        {
                                            ROS_INFO("imu befor turn left");
                                            continuousValue_x += 100;
                                            IMU_Value = get_IMU();
                                            IMU_theta = IMU_Modify();
                                            ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                            ROS_INFO("IMU_Value = %lf",IMU_Value);
                                            ROS_INFO("IMU_theta = %lf",IMU_theta);
                                            ROS_INFO("stay.theta + IMU_theta  = %lf",stay.theta + IMU_theta);
                                            ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                            tool->Delay(10);
                                        }*/
                                    }
                                    if(abs (IMU_Value) < 5){imu_ok_flag = true;}
                                    //imu_ok_flag = true;
                                    ROS_INFO("imu_ok_flag = true");
                                    if((imu_ok_flag == true) || (imu_ok_flag == false &&   Dx > -6) )// Dx > -7
                                    {
                                        //speed = def_speed();
                                        ROS_INFO("speed ++ in turn left");
                                        continuousValue_x += 100;
                                        turn_angle = def_turn_angle();
                                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                        ROS_INFO("turn_angle = %d",turn_angle);
                                        ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                        ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                        tool->Delay(10);
                                    }
                                }
                            }
                            else // 等速
                            {
                                if( /*Dx <= -7 &&  abs(IMU_Value) > 10 */(imu_ok_flag == false && Dx <= -6) ) //避障前用imu對正障礙物 Dx <= -7
                                {
                                    ROS_INFO("imu befor turn left");
                                    IMU_Value = get_IMU();
                                    IMU_theta = IMU_Modify();
                                    IMU_theta = IMU_angle_offest;
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("IMU_Value = %lf",IMU_Value);
                                    ROS_INFO("IMU_theta = %lf",IMU_theta);
                                    ROS_INFO("stay.theta + IMU_theta  = %lf",stay.theta + IMU_theta);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    tool->Delay(10);
                                    /*while(abs(IMU_Value) > 10 )
                                    {
                                        ROS_INFO("imu befor turn left");
                                        IMU_Value = get_IMU();
                                        IMU_theta = IMU_Modify();
                                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                        ROS_INFO("IMU_Value = %lf",IMU_Value);
                                        ROS_INFO("IMU_theta = %lf",IMU_theta);
                                        ROS_INFO("stay.theta + IMU_theta  = %lf",stay.theta + IMU_theta);
                                        ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                        tool->Delay(10);
                                    }*/
                                }
                                if(abs (IMU_Value) < 5){imu_ok_flag = true;}
                                //imu_ok_flag = true;
                                ROS_INFO("imu_ok_flag = true");
                                if((imu_ok_flag == true) || (imu_ok_flag == false &&  Dx > -6) ) //速度等於目前要求速度 保持速度旋轉 Dx > -7
                                {
                                    ROS_INFO("turn left");
                                    turn_angle = def_turn_angle();
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    tool->Delay(10);
                                }
                            }
                        }
                        else if(Dx == 0 || Dx == -31 ) //有障礙物 但還沒進焦點
                        {
                            if(continuousValue_x < speed) //速度小於目前要求速度 speed++ 保持角度
                            {
                                while(continuousValue_x < speed) 
                                {
                                    ROS_INFO("speed ++ obs not focus");
                                    //speed = def_speed();
                                    continuousValue_x += 100;
                                    turn_angle = def_turn_angle();
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    tool->Delay(10);
                                }
                            }
                            if(continuousValue_x > speed) //速度大於目前要求速度 speed-- 保持角度
                            {
                                while(continuousValue_x > speed) 
                                {
                                    ROS_INFO("speed -- obs not focus");
                                    //speed = def_speed();
                                    continuousValue_x -= 100;
                                    turn_angle = def_turn_angle();
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    tool->Delay(10);
                                }
                            }
                            else //速度等於目前要求速度 保持速度 保持角度
                            {
                                ROS_INFO("obs not focus");
                                turn_angle = def_turn_angle();
                                ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                ROS_INFO("turn_angle = %d",turn_angle);
                                ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
                                strategy_info->get_image_flag = true;
                                ros::spinOnce();
                                tool->Delay(10);
                            }
                        }
                        else if( ( (Dx < 17 && Dx > 14) ||(Dx < -14 && Dx > -17) )  ) //障礙物在面前一整片
                        {
                            if( l_center_Dy <= 14 && r_center_Dy <= 14)
                            {
                                ROS_INFO("Dx = %5f",Dx);
                                ROS_INFO("l_center_Dy = %d, r_center_Dy = %d",l_center_Dy,r_center_Dy);
                                ROS_INFO("ready to turnhead");
                                if(continuousValue_x > stay.x) //速度大於踏步
                                {
                                    while(continuousValue_x > stay.x)
                                    {
                                        if( abs(IMU_Value) > 5 )
                                        {
                                            ROS_INFO(" first imu fix before turnhead");
                                            continuousValue_x -= 100;
                                            IMU_Value = get_IMU();
                                            IMU_theta = IMU_Modify();strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                            tool->Delay(10);
                                            IMU_theta = IMU_angle_offest;
                                            ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                            ROS_INFO("IMU_Value = %lf",IMU_Value);
                                            ROS_INFO("IMU_theta = %lf",IMU_theta);
                                            ROS_INFO("stay.theta + IMU_theta  = %lf",stay.theta + IMU_theta);
                                            ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                            tool->Delay(10);
                                        }
                                        else
                                        {
                                            ROS_INFO("speed-- in turnhead");
                                            continuousValue_x -= 100;
                                            turn_angle = 0;
                                            ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                            ROS_INFO("turn_angle = 0",turn_angle);
                                            ROS_INFO("stay.theta + turn_angle = %d",stay.theta + 0);
                                            ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous);  
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                            tool->Delay(10);
                                        }
                                    }   
                                }
                                else if( abs(IMU_Value) > 5 ) //避障前用imu對正障礙物
                                {

                                    ROS_INFO("second imu fix before turnhead");
                                    IMU_Value = get_IMU();
                                    IMU_theta = IMU_Modify();
                                    IMU_theta = IMU_angle_offest;
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("IMU_Value = %lf",IMU_Value);
                                    ROS_INFO("IMU_theta = %lf",IMU_theta);
                                    ROS_INFO("stay.theta + IMU_theta  = %lf",stay.theta + IMU_theta);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    tool->Delay(10);
                                }
                                else  //進入轉頭
                                { 
                                    ROS_INFO("AVOID--->TURNHEAD");
                                    turnhead_flag = true;
                                    strategy_state = TURNHEAD;
                                }
                            }
                            else
                            {
                                if( WR > WL )
                                {
                                    ROS_INFO("turn right 2");
                                    //turn_angle = def_turn_angle();
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta - 8, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    tool->Delay(10);
                                }
                                else if( WL > WR )
                                {
                                    ROS_INFO("turn left 2");
                                    //turn_angle = def_turn_angle();
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + 8, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    tool->Delay(10);
                                }
                            }
                        }
                        else if(2 > Dx > -2)//2 > Dx > -2  障礙物在邊邊 17 > Dx > 14 || -14 > Dx > -17 
                        {
                            imu_ok_flag = false;
                            ROS_INFO("imu_ok_flag = false");
                            if( center_Dy == 0 )
                            {
                                if(continuousValue_x < maxspeed)//若小於最高速 speed++
                                {
                                    while(continuousValue_x < maxspeed) //
                                    {
                                        ROS_INFO("speed ++ in go straight");
                                        continuousValue_x += 100;
                                        //turn_angle = def_turn_angle();
                                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                        ROS_INFO("turn_angle = %d",turn_angle);
                                        ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                        ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + 0, IMU_continuous); 
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                        tool->Delay(10);
                                    }
                                }
                                else if(continuousValue_x == maxspeed) //保持目前角度直走
                                {   
                                    ROS_INFO("void speed up finish in go straight");
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + 0, IMU_continuous);  
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    tool->Delay(10);
                                    
                                }
                            }
                            else
                            {
                                if(continuousValue_x < maxspeed)//若小於最高速 speed++
                                {
                                    while(continuousValue_x < maxspeed) //
                                    {
                                        ROS_INFO("speed ++ in no avoid");
                                        continuousValue_x += 100;
                                        //turn_angle = def_turn_angle();
                                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                        ROS_INFO("turn_angle = %d",turn_angle);
                                        ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                        ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                        tool->Delay(10);
                                    }
                                }
                                else if(continuousValue_x == maxspeed) //保持目前角度直走
                                {   
                                    ROS_INFO("void speed up finish in no avoid");
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta , IMU_continuous);  
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    tool->Delay(10);
                                    
                                }
                            }
                        }
                    }
                    else if (Dy = 24 )//Dy = 24 no obs
                    {
                        speed = def_speed();
                        if(continuousValue_x < maxspeed)//若小於最高速 speed++
                        {
                            while(continuousValue_x < maxspeed) //
                            {
                                ROS_INFO("speed ++ in no obs");
                                continuousValue_x += 100;
                                IMU_Value = get_IMU();
                                IMU_theta = IMU_Modify();
                                IMU_theta = IMU_angle_offest;
                                //turn_angle = def_turn_angle();
                                ROS_INFO("IMU_Value = %lf",IMU_Value);
                                ROS_INFO("IMU_theta = %lf",IMU_theta);
                                ROS_INFO("stay.theta + IMU_theta  = %lf",stay.theta + IMU_theta);
                                ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous); 
                                strategy_info->get_image_flag = true;
                                ros::spinOnce();
                                tool->Delay(10);
                            }
                        }
                        else if(continuousValue_x == maxspeed) //保持目前角度直走
                        {   
                            IMU_Value = get_IMU();
                            IMU_theta = IMU_Modify();
                            IMU_theta = IMU_angle_offest;
                            ROS_INFO("void speed up finish in no obs");
                            ROS_INFO("continuousValue_x = %d",continuousValue_x);
                            ROS_INFO("turn_angle = %d",turn_angle);
                            ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                            ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + IMU_theta, IMU_continuous);  
                            strategy_info->get_image_flag = true;
                            ros::spinOnce();
                            tool->Delay(10);
                            
                        }
                    }
                }
                
                ROS_INFO("break");
            break;

            case TURNHEAD: //轉頭
                ROS_INFO("TURNHEAD");
                get_IMU();
                if(turnhead_flag == true)
                {
                    ROS_INFO("turnhead_flag == true");

                    turn_angle = 0;
                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                    ROS_INFO("turn_angle = %d",turn_angle);
                    ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                    ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous);  
                    strategy_info->get_image_flag = true;
                    ros::spinOnce();
                    tool->Delay(10);

                    ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1600, 300);             //低頭 1700
                    ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 1447, 300);           //頭像右轉
                    tool->Delay(1000);    
                    strategy_info->get_image_flag = true;       
                    ros::spinOnce();
                    tool->Delay(10);
                    /*ROS_INFO(" WR = %d",WR);
                    turn_WR = WR; //紀錄此刻之右權重
                    ROS_INFO(" turn_WR = %d",turn_WR);*/
                    if((in_reddoor_flag == true) && (RD != 0 && LD != 0)) //符合紅門條件
                    {
                        ROS_INFO(" Reddoor at rifht");
                        R_door_flag = true;
                        ROS_INFO(" R_door_flag = true");
                    }
                    else   //
                    {
                        ROS_INFO(" Deep_sum = %d",Deep_sum);
                        Deep_sum_R = Deep_sum; //紀錄此刻之右權重
                        ROS_INFO(" Deep_sum_R = %d",Deep_sum_R);
                    }
                    tool->Delay(100);


                    ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1600, 300);             //低頭 1700
                    ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2647, 300);           //頭像左轉
                    tool->Delay(1500);     
                    strategy_info->get_image_flag = true;      
                    ros::spinOnce();
                    tool->Delay(10);
                    /*ROS_INFO(" WL = %d",WL);
                    turn_WL = WL; //紀錄此刻之左權重
                    ROS_INFO(" turn_WL = %d",turn_WL);*/
                    if((in_reddoor_flag == true) && (RD != 0 && LD != 0)) //符合紅門條件
                    {
                        ROS_INFO(" Reddoor at left");
                        L_door_flag = true;
                        ROS_INFO(" L_door_flag = true");
                    }
                    else   //
                    {
                        ROS_INFO(" Deep_sum = %d",Deep_sum);
                        Deep_sum_L = Deep_sum; //紀錄此刻之右權重
                        ROS_INFO(" Deep_sum_L = %d",Deep_sum_L);
                    }
                    tool->Delay(100);
                    

                    ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1520, 300);             //頭回正常高度
                    ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300);           //頭轉回正中
                    tool->Delay(1000);
                    strategy_info->get_image_flag = true;      
                    ros::spinOnce();
                    tool->Delay(10);
                    
                    if( R_door_flag == true )
                    {
                        if( (in_reddoor_flag == false) )
                        {
                            while( l_center_Dy != 24 )
                            {
                                ROS_INFO("RIGHT_MOVE turnhead");
                                IMU_Value = get_IMU();
                                IMU_theta = IMU_Modify();
                                IMU_theta = IMU_angle_offest;
                                ROS_INFO("RightMove_T + IMU_theta  = %d",RightMove_T + IMU_theta);
                                ros_com->sendContinuousValue(RightMove_X, RightMove_Y, 0, RightMove_T + IMU_theta, IMU_continuous);
                                tool->Delay(100);
                                strategy_info->get_image_flag = true;
                                ros::spinOnce();
                                tool->Delay(10);
                            }
                            //strategy_state =  REDDOOR;
                        }
                        else
                        {
                            ROS_INFO("AVOID -> Reddoor");
                            strategy_state = REDDOOR;
                        }
                    }
                    else if( L_door_flag == true )
                    {
                        if( (in_reddoor_flag == false) )
                        {
                            while( r_center_Dy != 24 )
                            {
                                ROS_INFO("LEFT_MOVE turnhead");
                                IMU_Value = get_IMU();
                                IMU_theta = IMU_Modify();
                                IMU_theta = IMU_angle_offest;
                                ROS_INFO("LeftMove_T + IMU_theta  = %d",LeftMove_T + IMU_theta);
                                ros_com->sendContinuousValue(LeftMove_X, LeftMove_Y, 0,LeftMove_T + IMU_theta, IMU_continuous);
                                tool->Delay(100);
                                strategy_info->get_image_flag = true;
                                ros::spinOnce();
                                tool->Delay(10);
                            }
                            //strategy_state =  REDDOOR;
                        }
                        else
                        {
                            ROS_INFO("AVOID -> Reddoor");
                            strategy_state = REDDOOR;
                        }
                    }
                    else
                    {
                        if((Deep_sum_R - Deep_sum_L) > 5) //左權重大於右權重 代表缺口在右邊 //(turn_WL - turn_WR) > 100
                        {
                            ROS_INFO("turn right after turnhead");
                            turnhead_flag == false;
                            ROS_INFO("turnhead_flag == false");

                            if( abs(Dx) >= 2)
                            {
                                while( abs(Dx) >= 2)
                                {
                                    ROS_INFO("turn right in turnhead");
                                    //turn_angle = def_turn_angle();
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                    if(abs(IMU_Value) > 70) //若超過90度修正
                                    {
                                        ros_com->sendContinuousValue(Lmove.x, Lmove.y, 0, Lmove.theta + 5, IMU_continuous);
                                    }
                                    else
                                    {
                                        ros_com->sendContinuousValue(Rmove.x, Rmove.y, 0, Rmove.theta - 10, IMU_continuous); 
                                    }
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    tool->Delay(10);
                                }
                            }
                            if( abs(Dx) < 2 ) //0 < Dx < 2
                            {
                                ROS_INFO("finish turn right in turnhead");
                                turnhead_flag == false;
                                strategy_state = AVOID;
                            }
                        }
                        else if( (Deep_sum_L - Deep_sum_R) > 5 ) //右權重大於左權重 代表缺口在左邊 //(turn_WR - turn_WL) > 100 
                        {
                            ROS_INFO("turn left after turnhead");
                            turnhead_flag == false;
                            ROS_INFO("turnhead_flag == false");

                            if( abs(Dx) >= 2)
                            {
                                while( abs(Dx) >= 2)
                                {
                                    ROS_INFO("turn left in turnhead");
                                    //turn_angle = def_turn_angle();
                                    ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                    ROS_INFO("turn_angle = %d",turn_angle);
                                    ROS_INFO("stay.theta + turn_angle = %d",stay.theta + turn_angle);
                                    if(abs(IMU_Value) > 70) //若超過90度修正
                                    {
                                        ros_com->sendContinuousValue(Rmove.x, Rmove.y, 0, Rmove.theta - 5, IMU_continuous);
                                    }
                                    else
                                    {
                                        ros_com->sendContinuousValue(Lmove.x, Lmove.y, 0, Lmove.theta + 10, IMU_continuous); 
                                    }  
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    tool->Delay(10);
                                }
                            }
                            if( abs(Dx) < 2 ) //0 > Dx > -2
                            {
                                ROS_INFO("finish turn left in turnhead");
                                turnhead_flag == false;
                                strategy_state = AVOID;
                            }

                        }
                        else //imu > 80後做一般避障策略
                        {
                            ROS_INFO("abs(turn_WR - turn_WL) < 100 after turnhead");
                            turnhead_flag == false;
                            strategy_state = AVOID;
                        }
                    }
                }
                else //turnhead_flag == false 回一般避障策略
                {
                    turnhead_flag == false;
                    ROS_INFO("turnhead_flag == false");
                    strategy_state = AVOID;
                }
            break;   

            //0905++++++++++++++++
            case REDDOOR:
                if(in_reddoor_flag == true)
                {
                    ROS_INFO("state = REDDOOR");
                    ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1620, 300);
                    ROS_INFO("headup in reddor");
                    tool->Delay(500);

                    if( abs(slope_avg) > 0.05 )
                    {
                        //if( reddoor_slope_ok_flag == false)
                        //{
                            while( abs(slope_avg) > 0.05)
                            {
                                ROS_INFO("first slope fix in red");
                                slope();
                                ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                ROS_INFO("slope_avg = %lf",slope_avg);
                                ROS_INFO("angle_offest = %d",angle_offest);
                                //ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
                                strategy_info->get_image_flag = true;
                                ros::spinOnce();
                                tool->Delay(10);
                            }
                            reddoor_slope_ok_flag = true ;
                        //}
                    }
                    else if (abs(slope_avg) <= 0.05)
                    {
                        reddoor_slope_ok_flag = true ;
                    }
                
                    if( (Dy >= 4) && (redoor_dis == false))
                    {
                        ROS_INFO("distance too far 1");
                        continuousValue_x = stay.x + 1000;
                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                        ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta , IMU_continuous);  
                        tool->Delay(60);
                    }
                    else if( (Dy < 2) && (redoor_dis == false))
                    {
                        ROS_INFO("distance too short 1");
                        continuousValue_x = stay.x - 1000;
                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                        ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta , IMU_continuous);  
                        tool->Delay(60);
                    }
                    else
                    {
                        ROS_INFO("distance is ok 1");
                        redoor_dis = true;
                        // continuousValue_x = stay.x ;
                        // ROS_INFO("continuousValue_x = %d",continuousValue_x);
                        // ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta , IMU_continuous);
                        ROS_INFO("redoor_dis = true");
                    
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
                                ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous);  
                                tool->Delay(60);
                            }
                        }
                        else if(continuousValue_x == stay.x)
                        {
                            if( abs(slope_avg) > 0.05 )
                            {
                                //if( reddoor_slope_ok_flag == false)
                                //{
                                    while( abs(slope_avg) > 0.05)
                                    {
                                        ROS_INFO("slope fix in REDDOOR");
                                        slope();
                                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                                        ROS_INFO("slope_avg = %lf",slope_avg);
                                        ROS_INFO("angle_offest = %d",angle_offest);
                                        //ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                        tool->Delay(100);
                                    }
                                    reddoor_slope_ok_flag = true ;
                                //}
                            }
                            else if( abs(slope_avg) <= 0.05 )
                            {
                                reddoor_slope_ok_flag = true ;
                            }
                        }

                        if( abs(slope_avg) <= 0.05 )//再次確認斜率
                        {
                            if(RD < LD)             //對紅門做位移
                            {
                                ROS_INFO("LEFT_MOVE_1");
                                ros_com->sendContinuousValue(LeftMove_X, LeftMove_Y, 0,LeftMove_T, IMU_continuous);
                                tool->Delay(100);
                            }
                            else if(RD > LD)
                            {
                                ROS_INFO("RIGHT_MOVE_1");
                                ros_com->sendContinuousValue(RightMove_X, RightMove_Y, 0, RightMove_T, IMU_continuous);
                                tool->Delay(100);
                            }
                            else if(RD == LD)       //對下方藍模做比較
                            {   
                                if( /*(L_XMAX <= 65) || (R_XMIN <= 65) || */(LeftblueOBS_XMax < 65 && RightblueOBS_XMin > 260) )
                                {
                                    // if( Dy < 2 ) 
                                    // {
                                    //     //while( Dy < 3 )
                                    //     //{
                                    //         ROS_INFO("back back");
                                    //         continuousValue_x = stay.x - 1000;
                                    //         ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta , IMU_continuous);  
                                    //         tool->Delay(60);
                                    //     //}
                                    // }
                                    // else
                                    // {
                                        ROS_INFO("ready enter CRAWL;");
                                        strategy_state = CRAWL;
                                    //}
                                }
                                else if( (L_XMAX > 65) && (L_XMAX < 300)  )//(L_XMAX > 70 && L_XMAX < 300 ) || (LeftblueOBS_XMax > 50)
                                {
                                    ROS_INFO("RIGHT_MOVE");
                                    ROS_INFO("LeftblueOBS_XMax = %3d",LeftblueOBS_XMax);
                                    ros_com->sendContinuousValue(RightMove_X, RightMove_Y, 0, RightMove_T, IMU_continuous);
                                    tool->Delay(100);
                                }
                                else if( (R_XMIN > 65) && (R_XMIN < 300)  )//(R_XMIN > 70 && R_XMIN < 300 ) || (RightblueOBS_XMin < 270) 
                                { 
                                    ROS_INFO("LEFT_MOVE");
                                    ROS_INFO("RightblueOBS_XMin = %3d",RightblueOBS_XMin);
                                    ros_com->sendContinuousValue(LeftMove_X, LeftMove_Y, 0,LeftMove_T, IMU_continuous);
                                    tool->Delay(100);
                                }
                            } 
                        }
                        else
                        {
                            strategy_state = REDDOOR;
                        } 
                    }    
                }
                else 
                {
                    strategy_state = AVOID;
                }
                
                tool->Delay(100);
            
                //strategy_state = CRAWL;
            break;

            case CRAWL:
            ROS_INFO("state = CRAWL");

            reddoor_slope_ok_flag = false;
            if( abs(slope_avg) > 0.05 )
            {
                if( reddoor_slope_ok_flag == false)
                {
                    while( abs(slope_avg) > 0.05)
                    {
                        ROS_INFO("slope fix in crawl");
                        slope();
                        ROS_INFO("continuousValue_x = %d",continuousValue_x);
                        ROS_INFO("slope_avg = %lf",slope_avg);
                        ROS_INFO("angle_offest = %d",angle_offest);
                        //ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
                        strategy_info->get_image_flag = true;
                        ros::spinOnce();
                        tool->Delay(10);
                    }
                    reddoor_slope_ok_flag = true ;
                }
            }
            else if (abs(slope_avg) <= 0.05)
            {
                reddoor_slope_ok_flag = true ;
            }

            if( (Dy > 4) && (crawl_dis == false))
            {
                ROS_INFO("distance too far 2");
                continuousValue_x = stay.x + 1000;
                ROS_INFO("continuousValue_x = %d",continuousValue_x);
                ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta , IMU_continuous);  
                tool->Delay(60);
            }
            else if( (Dy <= 2) && (crawl_dis == false))
            {
                ROS_INFO("distance too short 2");
                continuousValue_x = stay.x - 1000;
                ROS_INFO("continuousValue_x = %d",continuousValue_x);
                ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta , IMU_continuous);  
                tool->Delay(60);
            }
            else
            {
                ROS_INFO("distance is ok 2");
                crawl_dis = true;
                continuousValue_x = stay.x ;
                ROS_INFO("continuousValue_x = %d",continuousValue_x);
                ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta , IMU_continuous);
                ROS_INFO("crawl_dis = true");

                if( abs(slope_avg) > 0.01 )
                {
                    //if( reddoor_slope_ok_flag == false)
                    //{
                        while( abs(slope_avg) > 0.01)
                        {
                            ROS_INFO("slope fix in crawl 2");
                            slope();
                            ROS_INFO("continuousValue_x = %d",continuousValue_x);
                            ROS_INFO("slope_avg = %lf",slope_avg);
                            ROS_INFO("angle_offest = %d",angle_offest);
                            //ros_com->sendContinuousValue(continuousValue_x, stay.y, 0, stay.theta + turn_angle, IMU_continuous); 
                            strategy_info->get_image_flag = true;
                            ros::spinOnce();
                            tool->Delay(10);
                        }
                        reddoor_slope_ok_flag = true ;
                    //}
                }
                else if (abs(slope_avg) <= 0.01)                        //爬行
                {
                    ROS_INFO(" ready crw");
                    ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous); 
                    tool->Delay(500);
                    ros_com->sendBodySector(29);
                    tool->Delay(500);
                    ros_com->sendHeadMotor(HeadMotorID::VerticalID, 2500, 600);
                    tool->Delay(100);
                    //ros_com->sendBodySector(5);
                    //tool->Delay(1000);
                    ros_com->sendBodySector(6);  //down
                    tool->Delay(3000);

                    for (int crwtime = 0; crwtime <= 30; crwtime++)
                    {
                        ROS_INFO("crw");
                        ROS_INFO("crw_up_flag = false");
                        strategy_info->get_image_flag = true;
                        ros::spinOnce();
                        for (int i = 0; i < strategy_info->color_mask_subject_cnts[2]; i++)
                        {
                            if (strategy_info->color_mask_subject[2][i].size > 32000)
                            {
                                ROS_INFO("stand up1");
                                crw_up_flag = true;
                                ROS_INFO("crw_up_flag = true");
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
                                crw_up_flag = true;
                                ROS_INFO("crw_up_flag = true");
                                break;
                            }
                        }
                        if (crw_up_flag == true)
                        {
                            ROS_INFO("ready stand");
                            break;
                        }
                        strategy_info->get_image_flag = true;
                        ros::spinOnce();

                        ros_com->sendBodySector(7); // crw
                        tool->Delay(2200);

                    }
                    tool->Delay(500);
                    ros_com->sendBodySector(8);  //stand up
                    tool->Delay(11000);
                    ros_com->sendBodySector(29);
                    tool->Delay(3000);
                    continuousValue_x = 0;
                    tool->Delay(3000);
                    strategy_state = AVOID;
                    // ROS_INFO("crw");
                    // ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous); 
                    // tool->Delay(50);
                    // tool->Delay(5000);
                }
            }
                
                tool->Delay(100);

                /*ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous); 
                tool->Delay(50);
                tool->Delay(5000);*/

                //strategy_state = AVOID;              //INIT or AVOID???
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

float KidsizeStrategy::get_IMU() //回傳imu值之副函式
{
    if (0 < strategy_info->getIMUValue().Yaw && strategy_info->getIMUValue().Yaw <= 180)
    {
        IMU_getValue = strategy_info->getIMUValue().Yaw;
    }
    else if (0 > strategy_info->getIMUValue().Yaw && strategy_info->getIMUValue().Yaw >= -179)
    {
        IMU_getValue = strategy_info->getIMUValue().Yaw;
    }
    ROS_INFO("IMU_getValue = %lf",IMU_getValue);
    return IMU_getValue;
}

int KidsizeStrategy::IMU_Modify() //用imu值判斷修正角度之副函式
{

    if (IMU_Value < 0)
    {
        if (abs(IMU_Value) >= 90)
        {
            IMU_angle_offest = 12;
        }
        else if (abs(IMU_Value) >= 45 && abs(IMU_Value) < 90)
        {
            IMU_angle_offest = 10;
        }
        else if (abs(IMU_Value) >= 30 && abs(IMU_Value) < 45)
        {
            IMU_angle_offest = 9;
        }
        else if (abs(IMU_Value) >= 15 && abs(IMU_Value) < 30)
        {
            IMU_angle_offest = 8;
        }
        else if (abs(IMU_Value) >= 10 && abs(IMU_Value) < 15)
        {
            IMU_angle_offest = 7;
        }
        else if (abs(IMU_Value) >= 5 && abs(IMU_Value) < 10)
        {
            IMU_angle_offest = 5;
        }
        else if (abs(IMU_Value) >= 2 && abs(IMU_Value) < 5)
        {
            IMU_angle_offest = 4;
        }
        else
        {
            IMU_angle_offest = 2;
        }
    }
    else if(IMU_Value > 0)
    {
        if (abs(IMU_Value) >= 90)
        {
            IMU_angle_offest = -12;
        }
        else if (abs(IMU_Value) >= 45 && abs(IMU_Value) < 90)
        {
            IMU_angle_offest = -10;
        }
        else if (abs(IMU_Value) >= 30 && abs(IMU_Value) < 45)
        {
            IMU_angle_offest = -9;
        }
        else if (abs(IMU_Value) >= 15 && abs(IMU_Value) < 30)
        {
            IMU_angle_offest = -8;
        }
        else if (abs(IMU_Value) >= 10 && abs(IMU_Value) < 15)
        {
            IMU_angle_offest = -7;
        }
        else if (abs(IMU_Value) >= 5 && abs(IMU_Value) < 10)
        {
            IMU_angle_offest = -5;
        }
        else if (abs(IMU_Value) >= 2 && abs(IMU_Value) < 5)
        {
            IMU_angle_offest = -4;
        }
        else
        {
            IMU_angle_offest = -2;
        }
    }
    ROS_INFO("(IMU_Modify)IMU_Value = %lf",IMU_Value);
    ROS_INFO("(IMU_Modify)IMU_theta = %3d",IMU_angle_offest);
    
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

int KidsizeStrategy::def_speed()
{
    if( (Dy >= 22 && Dy < 24) )
    {
        continuous_speed = stay.x + 4000;
    }
    else if( (Dy >= 20 && Dy < 22) )
    {
        continuous_speed = stay.x + 3500;
    }
    else if( (Dy >= 18 && Dy < 20) )
    {
        continuous_speed = stay.x + 3000;
    }
    else if( (Dy >= 16 && Dy < 18) )
    {
        continuous_speed = stay.x + 2500;
    }
    else if( (Dy >= 12 && Dy < 16) )
    {
        continuous_speed = stay.x + 2000;
    }
    else if( (Dy >= 10 && Dy < 12) )
    {
        continuous_speed = stay.x + 1500;
    }
    else if( (Dy >= 8 && Dy < 10) )
    {
        continuous_speed = stay.x + 1000;
    }
    else if( (Dy >= 6 && Dy < 8) )
    {
        continuous_speed = stay.x + 1000;
    }
    else if( (Dy >= 4 && Dy < 6) )
    {
        continuous_speed = stay.x + 500;
    }
    else if( (Dy >= 2 && Dy < 4) )
    {
        continuous_speed = stay.x ;
    }
    else if( (Dy >= 0 && Dy < 2) )
    {
        continuous_speed = stay.x ;
    }
    return continuous_speed;
}
int KidsizeStrategy::def_turn_angle() //用Dx判斷旋轉角度之副函式
{
    if(Dx > 0)                       //Dx > 0 -> obstacle in left -> turn right
    {
        ROS_INFO("Dx > 0   turn right");
        if(abs(Dx) <= 16 && abs(Dx) > 13)
        {
            //ROS_INFO("abs(x_boundary) < 16 && abs(x_boundary) > 11");
            continuous_angle_offset = -11;
        }
        else if(abs(Dx) <= 13 && abs(Dx) > 10)
        {
           // ROS_INFO("abs(x_boundary) < 13 && abs(x_boundary) > 10");
            continuous_angle_offset = -11;
        }
        else if(abs(Dx) <= 10 && abs(Dx) > 8)
        {
           // ROS_INFO("abs(x_boundary) < 13 && abs(x_boundary) > 10");
            continuous_angle_offset = -10;
        }
        else if(abs(Dx) <= 8 && abs(Dx) > 6)
        {
            //ROS_INFO("abs(x_boundary) < 10 && abs(x_boundary) > 7");
            continuous_angle_offset = -10;
        }
        else if(abs(Dx) <= 6 && abs(Dx) > 4)
        {
            //ROS_INFO("abs(x_boundary) < 10 && abs(x_boundary) > 7");
            continuous_angle_offset = -9;
        }
        else if(abs(Dx) <= 4 && abs(Dx) > 3)
        {
            //ROS_INFO("abs(x_boundary) < 10 && abs(x_boundary) > 7");
            continuous_angle_offset = -8;
        }
        else if(abs(Dx) <= 3 && abs(Dx) > 2)
        {
            //ROS_INFO("abs(x_boundary) < 10 && abs(x_boundary) > 7");
            continuous_angle_offset = -7;
        }
        else
        {
            //ROS_INFO("abs(Dx) < 1");
            continuous_angle_offset = -5;
        }
    }

    else if(Dx < 0)              //Dx < 0 -> obstacle in right -> turn left
    {
        ROS_INFO("Dx < 0  turn left");
        if(abs(Dx) <= 16 && abs(Dx) > 13)
        {
            continuous_angle_offset = 11;
        }
        else if(abs(Dx) <= 13 && abs(Dx) > 10)
        {
            continuous_angle_offset = 10;
        }
        else if(abs(Dx) <= 10 && abs(Dx) > 8)
        {
            continuous_angle_offset = 9;
        }
        else if(abs(Dx) <= 8 && abs(Dx) > 6)
        {
            continuous_angle_offset = 8;
        }
        else if(abs(Dx) <= 6 && abs(Dx) > 4)
        {
            continuous_angle_offset = 7;
        }
        else if(abs(Dx) <= 4 && abs(Dx) > 3)
        {
            continuous_angle_offset = 7;
        }
        else if(abs(Dx) <= 3 && abs(Dx) > 2)
        {
            continuous_angle_offset = 6;
        }
        else
        {
            continuous_angle_offset = 5;
        }   
    }
    else
    {
        continuous_angle_offset = 0;
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

void KidsizeStrategy::GetParameter(const strategy::GetParameter &msg) //GetParameter.msg之讀檔
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
    center_Dy = msg.center_Dy;
    one_b_flag = msg.one_b_flag;
    two_b_flag = msg.two_b_flag;
    Deep_sum = msg.Deep_sum;
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
        if (abs(slope_avg) > 0.6 )
        {
            angle_offest = 12;
            ros_com->sendContinuousValue(LeftSlope_X, LeftSlope_Y, 0,LeftSlope_T + angle_offest, IMU_continuous);
            tool->Delay(100);
        }
        else if (abs(slope_avg) > 0.4 && abs(slope_avg) <= 0.6)
        {
            angle_offest = 10;
            ros_com->sendContinuousValue(LeftSlope_X, LeftSlope_Y, 0,LeftSlope_T + angle_offest, IMU_continuous);
            tool->Delay(100);
        }
        else if (abs(slope_avg) > 0.3 && abs(slope_avg) <= 0.4)
        {
            angle_offest = 8;
            ros_com->sendContinuousValue(LeftSlope_X, LeftSlope_Y, 0,LeftSlope_T + angle_offest, IMU_continuous);
            tool->Delay(100);
        }
        else if (abs(slope_avg) > 0.2 && abs(slope_avg) <= 0.3)
        {
            angle_offest = 6;
            ros_com->sendContinuousValue(LeftSlope_X, LeftSlope_Y, 0,LeftSlope_T + angle_offest, IMU_continuous);
            tool->Delay(100);
        }
        else if (abs(slope_avg) > 0.1 && abs(slope_avg) <= 0.2)
        {
            angle_offest = 4;
            ros_com->sendContinuousValue(LeftSlope_X, LeftSlope_Y, 0,LeftSlope_T + angle_offest, IMU_continuous);
            tool->Delay(100);
        }
        else if (abs(slope_avg) > 0.05 && abs(slope_avg) <= 0.1)
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
        if (abs(slope_avg) > 0.6 )
        {
            angle_offest = -14;
            ros_com->sendContinuousValue(LeftSlope_X, LeftSlope_Y, 0,LeftSlope_T + angle_offest, IMU_continuous);
            tool->Delay(100);
        }
        else if (abs(slope_avg) > 0.4 && abs(slope_avg) <= 0.6)
        {
            angle_offest = -10;
            ros_com->sendContinuousValue(LeftSlope_X, LeftSlope_Y, 0,LeftSlope_T + angle_offest, IMU_continuous);
            tool->Delay(100);
        }
        else if (abs(slope_avg) > 0.3 && abs(slope_avg) <= 0.4)
        {
            angle_offest = -8;
            ros_com->sendContinuousValue(LeftSlope_X, LeftSlope_Y, 0,LeftSlope_T + angle_offest, IMU_continuous);
            tool->Delay(100);
        }
        else if (abs(slope_avg) > 0.2 && abs(slope_avg) <= 0.3)
        {
            angle_offest = -6;
            ros_com->sendContinuousValue(LeftSlope_X, LeftSlope_Y, 0,LeftSlope_T + angle_offest, IMU_continuous);
            tool->Delay(100);
        }
        else if (abs(slope_avg) > 0.1 && abs(slope_avg) <= 0.2)
        {
            angle_offest = -4;
            ros_com->sendContinuousValue(LeftSlope_X, LeftSlope_Y, 0,LeftSlope_T + angle_offest, IMU_continuous);
            tool->Delay(100);
        }
        else if (abs(slope_avg) > 0.05 && abs(slope_avg) <= 0.1)
        {
            angle_offest = -2;
            ros_com->sendContinuousValue(LeftSlope_X, LeftSlope_Y, 0,LeftSlope_T + angle_offest, IMU_continuous);
            tool->Delay(100);
        }
        else
        {
            angle_offest = 0;
        }
    }
}
//0905++++++++
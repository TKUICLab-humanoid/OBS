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
    int FocusMatrix[32] = {3, 3, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 3, 3}; //攝影機內之焦點矩陣
    int FocusMatrix_R[32] = {4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1};
    int FocusMatrix_L[32] = {1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4};
    int LeftMove[32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 21};
    int RightMove[32] = {21, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    if (strategy_info->getStrategyStart()) //策略指撥開啟
    {
        switch (m_state)
        {
        case P_INIT:
            printinfo();
            m_state_string = "P_INIT";
            readwalkinggait(); 
            load_dirtxt();

            if (!Continuous_flag) //起步步態
            {
                ros_com->sendBodySector(4); //動作磁區
                tool->Delay(1000);
                ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous); //ros_com->sendBodyAuto(-450, 0, 0,-3, WalkingMode::ContinuousStep,IMU_continuous);

                tool->Delay(1300);
                Continuous_flag = true;
            }
            if (dirmap[0])  
                first_move_flag = true;
            else
                first_move_flag = false;
            ////////////////////////////first move on/////////////////////////////////
            if (first_move_flag == true && first_act_flag == true) //first_move_flag = special case  , 第一層地圖為一排時使用
            {                                                      //配合Parameter -- firstmove.ini一起看比較好理解
               
                if (dirmap[1] == 1) 
                    walking_state = DIRmap_RIGHT;
                else if (dirmap[1] == 2)
                    walking_state = DIRmap_LEFT;
                m_state = P_WALKINGGAIT; 
                break;
            }
            ////////////////////////////first move off////////////////////////////////
            check_obs = false;                                          //確認障礙物
            check_road = false;                                         //確認路    
            checking_obs = false;                                       //正在確認障礙物(是一個狀態)
            compareObs = 0;                                             //存取深度矩陣之最小值
            compareObssize = 0;                                         //障礙物面積    
            head_direction = RHD_Center;                                //頭的上一個狀態在中間
            turnslope_flag = false;                                     //遇到障礙物就正對
            slope_flag = false;                                         //turnslope_flag內使用 
            obs_data.size = 0;                                          //宣告"obs_data.size"內的值......"儲存面積的值"
            continuousValue_x = 0;                                      //初速度 = 0
            insideFMcnt = 0;                                            //有幾行為危險區(深度於焦點內)
            in_reddoor_flag = false;                                    //add
            sideline_zero_flag = true;                                  //add
            ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1603, 300); //頭部馬達刻度(上下)
            tool->Delay(100);
            ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1603, 300);
            tool->Delay(100);
            ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1603, 300);
            tool->Delay(100);
            ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300); //頭部馬達刻度（左右）左正右負
            tool->Delay(100);
            ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300);
            tool->Delay(100);
            ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300);

            tool->Delay(1000);
            stand_flag = true;                    //站起來
            strategy_info->get_image_flag = true; //擷取影像
            first_continuous_flag = false;        // "< 20" 內，第一步進左移或右移的旗標，就繼續走下去，不要左右走一直卡在障礙物前  
            Ry_fastest = dirdata[34];             //將讀檔內的值丟掉自訂變數內  
            Ly_fastest = dirdata[37];            
            m_obs_vector.clear();
            m_state = P_MATRIX_CALCULATE;
            switch (dirdata[39]) //連續步態之IMU
            {
            case 1:
                IMU_continuous = SensorMode::None; //全關
                break;

            case 2:
                IMU_continuous = SensorMode::Roll; //只開y方向
                break;

            case 3:
                IMU_continuous = SensorMode::Pitch; //只開x方向
                break;

            case 4:
                IMU_continuous = SensorMode::RollPitch; //全開
                break;
            }
            switch (dirdata[46]) //單步步態之IMU
            {
            case 1:
                IMU_single = SensorMode::None;
                break;

            case 2:
                IMU_single = SensorMode::Roll;
                break;

            case 3:
                IMU_single = SensorMode::Pitch;
                break;

            case 4:
                IMU_single = SensorMode::RollPitch;
                break;
            }
            printinfo();
            break;

        case P_MATRIX_CALCULATE: 
            printinfo();
            m_state_string = "P_MATRIX_CALCULATE";
            compareObs = 10000;
            obs_data.size = 0;
            insideFMcnt = 0;
            m_obs_vector.clear(); //深度矩陣內...沒有障礙物(clear)  
            check_obs = false;    
            checking_obs = false; 
            zero_flag = true;     //zero_flag = 直走flag
            strategy_info->get_image_flag = true;

            for (int i = 0; i < DeepMatrixSize; i++) //深度矩陣之運算    
            {
                if (FocusMatrix[i] - DeepMatrixValue[i] > 0)
                {
                    insideFMcnt++;                                         
                    FilterMatrix[i] = FocusMatrix[i] - DeepMatrixValue[i]; 
                    if (check_obs == false)                                
                    {
                        obs_data.x_min = i;  
                        check_obs = true;    
                        checking_obs = true; 
                    }
                    if (checking_obs == true) 
                    {
                        obs_data.size += FilterMatrix[i];    
                        if (DeepMatrixValue[i] < compareObs) 
                        {
                            compareObs = DeepMatrixValue[i];
                        }
                    }
                    if (zero_flag == true) 
                    {
                        zero_flag = false; 
                    }
                }
                else 
                {
                    if (check_obs == true)
                    {                           
                        obs_data.x_max = i - 1; 
                        checking_obs = false;
                        check_obs = false;
                        obs_data.y = compareObs;
                        obs_data.x = (obs_data.x_max + obs_data.x_min) / 2;
                        m_obs_vector.push_back(obs_data); 
                        obs_data.size = 0;
                        compareObs = 10000;
                    }
                    FilterMatrix[i] = 0; 
                }
                if (i == 31 && check_obs == true) 
                {                                 
                    obs_data.x_max = i;           
                    check_obs = false;
                    obs_data.y = compareObs;
                    obs_data.x = (obs_data.x_max + obs_data.x_min) / 2; //x值之中點
                    m_obs_vector.push_back(obs_data);
                    compareObs = 10000;
                    obs_data.size = 0;
                }
            }
            m_finish_obs_vector.clear();
            if (m_obs_vector.size() > 1) 
            {
                for (int i = 0; i < m_obs_vector.size() - 1; i++) //判斷兩個障礙物之間的洞的距離
                {
                    int x_min = m_obs_vector[i + 1].x_min;
                    int x_max = m_obs_vector[i].x_max;
                    if (i == 0)
                    {
                        obs_data.size = m_obs_vector[0].size;
                    }
                    if (abs(m_obs_vector[i + 1].x_min - m_obs_vector[i].x_max) < 20) //兩障礙物的邊相減 < 20（20/32） 
                    {
                        ///ROS_INFO("NO Road");
                        first_continuous_flag = true; 

                        ///ROS_INFO(">2obs");
                        m_state = P_FM_TURNHEAD;             
                        if (continuousValue_x > dirdata[30]) 
                        {           
                            while (continuousValue_x > dirdata[30]) //直走轉平移之減速區間
                            {                                       
                                continuousValue_x -= 50;            //注意如果一次減百位數 讀檔端不能有無法被整除的數出現
                                IMUSlope();
                                FaceToFinialLineFun();                                                                                                 //計算有無正對障礙物之副函式  
                                ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); //連續步態的值  
                                tool->Delay(50);                                                                                                       
                                strategy_info->get_image_flag = true;                                                                                  
                                ros::spinOnce();
                            }
                        }
                        else 
                        {
                            while (continuousValue_x < dirdata[30]) //直走轉平移之加速區間
                            {
                                continuousValue_x += 50;                                                                                               
                                IMUSlope();                                                                                                            
                                FaceToFinialLineFun();                                                                                                 
                                ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); //連續步態的值
                                tool->Delay(100);
                                strategy_info->get_image_flag = true; 
                                ros::spinOnce();
                            }
                        }
                        ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 1447, 600); 
                        tool->Delay(100);
                        ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 1447, 600);
                        tool->Delay(100);
                        ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 1447, 600);
                        tool->Delay(800);
                        strategy_info->get_image_flag = true;
                        ros::spinOnce();
                        true_RMoveValue = 0;
                        true_LMoveValue = 0;
                        head_direction = RHD_Center; //頭在上一刻的狀態   
                        Turnhead_flag = true;
                        break;
                    }
                    else //計算障礙物的各種資訊
                    {
                        obs_data.x_min = m_obs_vector[i].x_min;
                        obs_data.x_max = m_obs_vector[i].x_max;
                        obs_data.x = m_obs_vector[i].x;
                        obs_data.y = m_obs_vector[i].y;
                        obs_data.size = m_obs_vector[i].size;
                        m_finish_obs_vector.push_back(obs_data); 
                        obs_data.x_min = m_obs_vector[i + 1].x_min;
                        obs_data.x_max = m_obs_vector[i + 1].x_max;
                        obs_data.x = m_obs_vector[i + 1].x;
                        obs_data.y = m_obs_vector[i + 1].y;
                        obs_data.size = m_obs_vector[i + 1].size;
                        if (i == m_obs_vector.size() - 2) 
                        {
                            m_finish_obs_vector.push_back(obs_data); 
                        }
                    }
                }
            }
            else
            {
                if (m_obs_vector.size() == 1) 
                {
                    obs_data.x_min = m_obs_vector[0].x_min;
                    obs_data.x_max = m_obs_vector[0].x_max;
                    obs_data.x = m_obs_vector[0].x;
                    obs_data.y = m_obs_vector[0].y;
                    obs_data.size = m_obs_vector[0].size;
                    m_finish_obs_vector.push_back(obs_data);                                                           
                    if ((m_finish_obs_vector[0].x_min == 0 && m_finish_obs_vector[0].x_max == 31) || insideFMcnt > 29) 
                    {
                        //ROS_INFO("Full!!");
                        if (continuousValue_x > dirdata[30]) 
                        {
                            while (continuousValue_x > dirdata[30]) 
                            {
                                continuousValue_x -= 50; 
                                IMUSlope();
                                FaceToFinialLineFun();                                                                                                 
                                ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                tool->Delay(50);
                                strategy_info->get_image_flag = true; 
                                ros::spinOnce();
                            }
                        }
                        else 
                        {
                            while (continuousValue_x < dirdata[30]) 
                            {
                                continuousValue_x += 50;                                                                                               
                                IMUSlope();                                                                                                            
                                FaceToFinialLineFun();                                                                                                 
                                ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                tool->Delay(100);
                                strategy_info->get_image_flag = true; 
                                ros::spinOnce();
                            }
                        }

                        m_state = P_FM_TURNHEAD;                                      
                        ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 1447, 600); 
                        tool->Delay(100);
                        ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 1447, 600);
                        tool->Delay(100);
                        ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 1447, 600);
                        tool->Delay(800);
                        strategy_info->get_image_flag = true;
                        ros::spinOnce();
                        true_RMoveValue = 0;
                        true_LMoveValue = 0;
                        head_direction = RHD_Center; 
                        Turnhead_flag = true;
                        insideFMcnt = 0;
                        break;
                    }
                }
            }
            if (strategy_info->color_mask_subject_cnts[5] != 0) //障礙物為紅色    
            {
                for (int i = 0; i < strategy_info->color_mask_subject_cnts[5]; i++) 
                {
                    if (strategy_info->color_mask_subject[5][i].size > 1000) //紅色面積 > 1000    
                    {
                        //ROS_INFO("P_DOOR");
                        continuousValue_x = dirdata[30]; //注意如果一次減百位數 讀檔端不能有無法被整除的數出現
                        IMUSlope();
                        FaceToFinialLineFun(); //計算有無正對障礙物之副函式 
                        ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous);
                        m_state = P_DOOR;                                           
                        ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1783, 300); 
                        tool->Delay(100);
                        ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1783, 300);
                        tool->Delay(100);
                        ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300); 
                        tool->Delay(100);
                        ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 300);
                        tool->Delay(2000);
                        strategy_info->get_image_flag = true; 
                        ros::spinOnce();
                        break;
                    }
                }
            }

            if (Turnhead_flag == true && m_state != P_DOOR) 
            {
                //ROS_INFO("breakout");
                break;
            }
            if (m_state != P_DOOR)
            {
                if (zero_flag == true) 
                {          
                    if (walking_state == continuousValue_Ry) //平移轉直走之減速區間  
                    {
                        while(dirdata[34] != dirdata[31])
                            {
                                dirdata[34] += 100;
                                IMUSlope();
                                traverse();                 
                                ros_com->sendContinuousValue(dirdata[33], dirdata[34], 0, dirdata[35] + continous_angle_offest,IMU_continuous);    
                                tool->Delay(100);
                                strategy_info->get_image_flag = true;    
                                ros::spinOnce();
                            }
                    }
                    else if (walking_state == continuousValue_Ly) 
                    {
                        while(dirdata[37] != dirdata[31])
                            {
                                dirdata[37] -= 100;
                                IMUSlope();
                                traverse();
                                ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest,IMU_continuous);    
                                tool->Delay(100);
                                strategy_info->get_image_flag = true;   
                                ros::spinOnce();
                            }
                    }
                    if (continuousValue_x < 3200) //直走加速區間
                    {
                        continuousValue_x += 100;
                        IMUSlope();
                        FaceToFinialLineFun();
                        ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                        tool->Delay(100);
                        strategy_info->get_image_flag = true;
                        ros::spinOnce();
                        sideline();
                    }
                    else
                    {
                        IMUSlope();
                        FaceToFinialLineFun();
                        ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                        tool->Delay(100);
                        strategy_info->get_image_flag = true;
                        ros::spinOnce();
                        sideline();
                    }
                    m_state = P_MATRIX_CALCULATE; 
                    break;
                }
                else
                {
                    m_state = P_FIND_WALKINGSTATE; 
                }
            }
            printinfo();
            break;                
        case P_FIND_WALKINGSTATE: 
            printinfo();
            m_state_string = "P_FIND_WALKINGSTATE";
            RMoveValue = 0;                             //畫面中右權重值
            LMoveValue = 0;                             //畫面中左權重值
            for (int i = 3; i < 28; i++) 
            {
                if (FilterMatrix[i] != 0)
                {
                    check_road = false;
                    break;
                }
                check_road = true; 
            }
            if (check_road == true) 
            {
                if (walking_state == continuousValue_Ry)
                {
                    while(dirdata[34] != dirdata[31])         
                        {
                            dirdata[34] += 100;         
                            IMUSlope();
                            traverse();
                            ros_com->sendContinuousValue(dirdata[33], dirdata[34], 0, dirdata[35] + continous_angle_offest,IMU_continuous);  
                            tool->Delay(100);
                            strategy_info->get_image_flag = true;
                            ros::spinOnce();
                        }
                }
                else if (walking_state == continuousValue_Ly)
                {
                    while(dirdata[37] != dirdata[31])        
                        {
                            dirdata[37] -= 100;        
                            IMUSlope();
                            traverse();
                            ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest,IMU_continuous);
                            tool->Delay(100);
                            strategy_info->get_image_flag = true;
                            ros::spinOnce();
                        }
                }
                if (continuousValue_x < 3200)
                {
                    continuousValue_x += 100; 
                    IMUSlope();
                    FaceToFinialLineFun();
                    ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                    tool->Delay(100);
                    strategy_info->get_image_flag = true;
                    ros::spinOnce();
                }
                else
                {
                    IMUSlope();
                    traverse();
                    ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                    tool->Delay(100);
                    strategy_info->get_image_flag = true;
                    ros::spinOnce();
                }
                m_state = P_MATRIX_CALCULATE;
                break;
            }
            for (int i = 0; i < DeepMatrixSize; i++)
            {
                RMoveValue += RightMove[i] * FilterMatrix[i]; 
                LMoveValue += LeftMove[i] * FilterMatrix[i];
            }
            if (abs(LMoveValue - RMoveValue) < 8 && LMoveValue > 10 && RMoveValue > 10) 
            {
                //ROS_INFO("<20");
                if (continuousValue_x > dirdata[30])
                {
                    while (continuousValue_x > dirdata[30])
                    {
                        continuousValue_x -= 50; 
                        IMUSlope();
                        FaceToFinialLineFun();
                        ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                        tool->Delay(50);
                        strategy_info->get_image_flag = true;
                        ros::spinOnce();
                    }
                }
                else 
                {
                    while (continuousValue_x < dirdata[30]) 
                    {
                        continuousValue_x += 50; 
                        IMUSlope();
                        FaceToFinialLineFun();
                        ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                        strategy_info->get_image_flag = true;
                        ros::spinOnce();
                    }
                }
                m_state = P_FM_TURNHEAD;                                      
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 1447, 600); 
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 1447, 600);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 1447, 600);
                tool->Delay(800);
                strategy_info->get_image_flag = true; 
                ros::spinOnce();
                head_direction = RHD_Center; 
                Turnhead_flag = true;
                true_RMoveValue = 0;
                true_LMoveValue = 0;
                break;
            }
            else 
            {
                if (LMoveValue < RMoveValue) 
                {
                    //ROS_INFO("RMove");
                    if (m_finish_obs_vector.size() == 1) 
                    {
                        if (m_finish_obs_vector[0].y > 8) //距離障礙物的y值 > 8時開始減速    
                        {
                            if (walking_state == continuousValue_Ry)
                            {
                                while(dirdata[34] != dirdata[31])    
                                    {
                                        dirdata[34] += 100;
                                        IMUSlope();
                                        traverse();
                                        ros_com->sendContinuousValue(dirdata[33], dirdata[34], 0, dirdata[35] + continous_angle_offest,IMU_continuous);
                                        tool->Delay(100);
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                    }
                            }
                            else if (walking_state == continuousValue_Ly)
                            {
                                while(dirdata[37] != dirdata[31])   
                                    {
                                        dirdata[37] -= 100;  
                                        IMUSlope();
                                        traverse();
                                        ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest,IMU_continuous);
                                        tool->Delay(100);
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                    }
                            }
                            if (continuousValue_x > 900) 
                            {
                                continuousValue_x -= 100; 
                                IMUSlope();
                                FaceToFinialLineFun();
                                ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                strategy_info->get_image_flag = true;
                                ros::spinOnce();
                            }
                            else 
                            {
                                continuousValue_x += 100; 
                                IMUSlope();
                                FaceToFinialLineFun();
                                ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                strategy_info->get_image_flag = true;
                                ros::spinOnce();
                            }
                            m_state = P_MATRIX_CALCULATE; 
                        }
                        else 
                        {
                            if (m_finish_obs_vector[0].y > 5) //距離障礙物的y值 > 5時開始減速    
                            {
                                if (walking_state == continuousValue_Ry)
                                {
                                    while(dirdata[34] != dirdata[31])            
                                        {
                                            dirdata[34] += 100;
                                            IMUSlope();
                                            traverse(); 
                                            ros_com->sendContinuousValue(dirdata[33], dirdata[34], 0, dirdata[35] + continous_angle_offest,IMU_continuous);
                                            tool->Delay(100);
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                        }
                                }
                                else if (walking_state == continuousValue_Ly)
                                {
                                    while(dirdata[37] != dirdata[31])            
                                        {
                                            dirdata[37] -= 100;
                                            IMUSlope();
                                            traverse(); 
                                            ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest,IMU_continuous);
                                            tool->Delay(100);
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                        }
                                }
                                if (continuousValue_x > 600) 
                                {
                                    continuousValue_x -= 100;
                                    IMUSlope();
                                    FaceToFinialLineFun();
                                    ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                }
                                else
                                {
                                    continuousValue_x += 100;
                                    IMUSlope();
                                    FaceToFinialLineFun();
                                    ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                }
                                m_state = P_MATRIX_CALCULATE; 
                            }
                            else 
                            {
                                walking_state = continuousValue_Ry; 
                                m_state = P_WALKINGGAIT;
                            }
                        }
                        break;
                    }
                    else
                    {
                        for (int i = 0; i < m_finish_obs_vector.size(); i++) 
                        {
                            if (m_finish_obs_vector[i].y <= 10 && m_finish_obs_vector[i].size > compareObssize) 
                            {                                                                                  
                                compareObssize = m_finish_obs_vector[i].size;                                  
                                bigobs = i;                                                                     
                            }
                        }
                        if (m_finish_obs_vector[bigobs].y > 8) 
                        {
                            if (walking_state == continuousValue_Ry) 
                            {
                                while(dirdata[34] != dirdata[31])                  
                                        {
                                            dirdata[34] += 100;
                                            IMUSlope();
                                            traverse(); 
                                            ros_com->sendContinuousValue(dirdata[33], dirdata[34], 0, dirdata[35] + continous_angle_offest,IMU_continuous);
                                            tool->Delay(100);
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                        } 
                            }
                            else if (walking_state == continuousValue_Ly)
                            {
                                while(dirdata[37] != dirdata[31])                   
                                        {
                                            dirdata[37] -= 100;
                                            IMUSlope();
                                            traverse(); 
                                            ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest,IMU_continuous);
                                            tool->Delay(100);
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                        }
                            }
                            if (continuousValue_x > 900) 
                            {
                                continuousValue_x -= 100; 
                                IMUSlope();
                                FaceToFinialLineFun();
                                ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                strategy_info->get_image_flag = true;
                                ros::spinOnce();
                            }
                            else
                            {
                                continuousValue_x += 100;
                                IMUSlope();
                                FaceToFinialLineFun();
                                ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                strategy_info->get_image_flag = true;
                                ros::spinOnce();
                            }
                            m_state = P_MATRIX_CALCULATE; 
                        }
                        else
                        {
                            if (m_finish_obs_vector[bigobs].y > 5) 
                            {
                                if (walking_state == continuousValue_Ry)
                                {
                                    while(dirdata[34] != dirdata[31])
                                        {
                                            dirdata[34] += 100;
                                            IMUSlope();
                                            traverse(); 
                                            ros_com->sendContinuousValue(dirdata[33], dirdata[34], 0, dirdata[35] + continous_angle_offest,IMU_continuous);
                                            tool->Delay(100);
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                        }
                                }
                                else if (walking_state == continuousValue_Ly)
                                {
                                    while(dirdata[37] != dirdata[31])
                                        {
                                            dirdata[37] -= 100;
                                            IMUSlope();
                                            traverse(); 
                                            ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest,IMU_continuous);
                                            tool->Delay(100);
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                        }
                                }
                                if (continuousValue_x > 600) 
                                {
                                    continuousValue_x -= 100; 
                                    IMUSlope();
                                    FaceToFinialLineFun();
                                    ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                }
                                else
                                {
                                    continuousValue_x += 100; 
                                    IMUSlope();
                                    FaceToFinialLineFun();
                                    ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                }
                                m_state = P_MATRIX_CALCULATE; 
                            }
                            else 
                            {

                                walking_state = continuousValue_Ry; 
                                m_state = P_WALKINGGAIT; 
                            }
                        }
                        break;
                    }
                }
                else 
                {
                    //ROS_INFO("LMove");
                    if (m_finish_obs_vector.size() == 1) 
                    {
                        if (m_finish_obs_vector[0].y > 8) 
                        {
                            if (walking_state == continuousValue_Ry)
                            {
                                while(dirdata[34] != dirdata[31])                 
                                        {
                                            dirdata[34] += 100;
                                            IMUSlope();
                                            traverse(); 
                                            ros_com->sendContinuousValue(dirdata[33], dirdata[34], 0, dirdata[35] + continous_angle_offest,IMU_continuous);
                                            tool->Delay(100);
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                        }
                            }
                            else if (walking_state == continuousValue_Ly)
                            {
                                while(dirdata[37] != dirdata[31])                 
                                        {
                                            dirdata[37] -= 100;
                                            IMUSlope();
                                            traverse(); 
                                            ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest,IMU_continuous);
                                            tool->Delay(100);
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                        }
                            }
                            if (continuousValue_x > 900)
                            {
                                continuousValue_x -= 100;
                                IMUSlope();
                                FaceToFinialLineFun();
                                ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                strategy_info->get_image_flag = true;
                                ros::spinOnce();
                            }
                            else
                            {
                                continuousValue_x += 100;
                                IMUSlope();
                                FaceToFinialLineFun();
                                ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                strategy_info->get_image_flag = true;
                                ros::spinOnce();
                            }
                            m_state = P_MATRIX_CALCULATE; 
                        }
                        else
                        {
                            if (m_finish_obs_vector[0].y > 5) 
                            {
                                if (walking_state == continuousValue_Ry)
                                {
                                    while(dirdata[34] != dirdata[31])
                                            {
                                                dirdata[34] += 100;             
                                                IMUSlope();
                                                traverse(); 
                                                ros_com->sendContinuousValue(dirdata[33], dirdata[34], 0, dirdata[35] + continous_angle_offest,IMU_continuous);
                                                tool->Delay(100);
                                                strategy_info->get_image_flag = true;
                                                ros::spinOnce();
                                            }
                                }
                                else if (walking_state == continuousValue_Ly)
                                {
                                    while(dirdata[37] != dirdata[31])
                                            {
                                                dirdata[37] -= 100;             
                                                IMUSlope();
                                                traverse(); 
                                                ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest,IMU_continuous);
                                                tool->Delay(100);
                                                strategy_info->get_image_flag = true;
                                                ros::spinOnce();
                                            }
                                }
                                if (continuousValue_x > 600)
                                {
                                    continuousValue_x -= 100;
                                    IMUSlope();
                                    FaceToFinialLineFun();
                                    ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                }
                                else
                                {
                                    continuousValue_x += 100;
                                    IMUSlope();
                                    FaceToFinialLineFun();
                                    ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                }
                                m_state = P_MATRIX_CALCULATE; 
                            }
                            else 
                            {

                                walking_state = continuousValue_Ly; 
                                m_state = P_WALKINGGAIT; 
                            }
                        }
                        break;
                    }
                    else 
                    {
                        for (int i = 0; i < m_finish_obs_vector.size(); i++) 
                        {
                            if (m_finish_obs_vector[i].y <= 10 && m_finish_obs_vector[i].size > compareObssize) 
                            {                                                                                   
                                compareObssize = m_finish_obs_vector[i].size;                                   
                                bigobs = i;                                                                    
                            }
                        }
                        if (m_finish_obs_vector[bigobs].y > 8)
                        {
                            if (walking_state == continuousValue_Ry)
                            {
                                while(dirdata[34] != dirdata[31])
                                        {
                                            dirdata[34] += 100;
                                            IMUSlope();
                                            traverse(); 
                                            ros_com->sendContinuousValue(dirdata[33], dirdata[34], 0, dirdata[35] + continous_angle_offest,IMU_continuous);
                                            tool->Delay(100);
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                        }
                            }
                            else if (walking_state == continuousValue_Ly)
                            {
                                while(dirdata[37] != dirdata[31])
                                        {
                                            dirdata[37] -= 100;
                                            IMUSlope();
                                            traverse(); 
                                            ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest,IMU_continuous);
                                            tool->Delay(100);
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                        }
                            }
                            if (continuousValue_x > 900)
                            {
                                continuousValue_x -= 100;
                                IMUSlope();
                                FaceToFinialLineFun();
                                ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                strategy_info->get_image_flag = true;
                                ros::spinOnce();
                            }
                            else
                            {
                                continuousValue_x += 100;
                                IMUSlope();
                                FaceToFinialLineFun();
                                ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                strategy_info->get_image_flag = true;
                                ros::spinOnce();
                            }
                            m_state = P_MATRIX_CALCULATE; 
                        }
                        else
                        {
                            if (m_finish_obs_vector[bigobs].y > 5) 
                            {
                                if (walking_state == continuousValue_Ry)
                                {
                                    while(dirdata[34] != dirdata[31])
                                        {
                                            dirdata[34] += 100;
                                            IMUSlope();
                                            traverse(); 
                                            ros_com->sendContinuousValue(dirdata[33], dirdata[34], 0, dirdata[35] + continous_angle_offest,IMU_continuous);
                                            tool->Delay(100);
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                        }
                                }
                                else if (walking_state == continuousValue_Ly)
                                {
                                    while(dirdata[37] != dirdata[31])
                                        {
                                            dirdata[37] -= 100;
                                            IMUSlope();
                                            traverse(); 
                                            ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest,IMU_continuous);
                                            tool->Delay(100);
                                            strategy_info->get_image_flag = true;
                                            ros::spinOnce();
                                        }
                                }
                                if (continuousValue_x > 600)
                                {
                                    continuousValue_x -= 100;
                                    IMUSlope();
                                    FaceToFinialLineFun();
                                    ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                }
                                else
                                {
                                    continuousValue_x += 100;
                                    IMUSlope();
                                    FaceToFinialLineFun();
                                    ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                }
                                m_state = P_MATRIX_CALCULATE; 
                            }
                            else 
                            {
                                if (continuousValue_x > dirdata[30]) 
                                {
                                    while (continuousValue_x > dirdata[30]) 
                                    {
                                        continuousValue_x -= 50; 
                                        IMUSlope();
                                        FaceToFinialLineFun();
                                        ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                        tool->Delay(50);
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                    }
                                }
                                else 
                                {
                                    while (continuousValue_x < dirdata[30])
                                    {
                                        continuousValue_x += 50;
                                        IMUSlope();
                                        FaceToFinialLineFun();
                                        ros_com->sendContinuousValue(continuousValue_x, dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous); 
                                        tool->Delay(100);
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                    }
                                }
                                walking_state = continuousValue_Ly; 
                                m_state = P_WALKINGGAIT; 
                            }
                        }
                        break;
                    }
                }
            }
            printinfo();
            break;          
        case P_FM_TURNHEAD: 
            printinfo();
            m_state_string = "P_FM_TURNHEAD";
            switch (head_direction) 
            {
            case RHD_Center: //對障礙物辨識以及計算      
                for (int i = 0; i < DeepMatrixSize; i++) 
                {
                    true_RMoveValue += DeepMatrixValue[i] * DeepMatrixValue[i]; 
                }
                sideline();
                if (rightsidelinewarning == true)
                {
                    true_RMoveValue = 0;
                    printinfo();
                }
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2647, 600); 
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2647, 600); 
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2647, 600);
                tool->Delay(1600);
                strategy_info->get_image_flag = true; 
                ros::spinOnce();
                head_direction = RHD_Right;
                break;       
            case RHD_Right:  
                for (int i = 0; i < DeepMatrixSize; i++)
                {
                    true_LMoveValue += DeepMatrixValue[i] * DeepMatrixValue[i]; 
                }
                sideline();
                if (leftsidelinewarning == true)
                {
                    true_LMoveValue = 0;
                    printinfo();
                }
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 600);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 600); 
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 600);
                tool->Delay(800);
                head_direction = RHD_Left;
                break;
            case RHD_Left: 
                if (abs(true_LMoveValue - true_RMoveValue) < 10)
                {
                    strategy_info->get_image_flag = true;
                    ros::spinOnce();
                    IMUSlope();
                    traverse(); 
                    ros_com->sendContinuousValue(dirdata[30], dirdata[31], 0, dirdata[32] + continous_angle_offest, IMU_continuous);
                    m_state = P_WALKINGGAIT;
                }
                else if (true_LMoveValue < true_RMoveValue) 
                {
                    dirdata[34] = 0;
                    Turnhead_flag = false;              
                    walking_state = continuousValue_Ry; 
                    for (int j = 0;; j++)
                    {
                        printinfo();
                        if (j % 100 == 0)
                        {
                            hole_flag = true;
                            strategy_info->get_image_flag = true; 
                            ros::spinOnce();
                            tool->Delay(800);
                            for (int i = 17; i < 27; i++)
                            {
                                if (DeepMatrixValue[i] < 20)
                                {
                                    hole_flag = false;
                                    break;
                                }
                            }
                            sideline();
                            if (leftsidelinewarning == true || rightsidelinewarning == true)
                            {
                                if (hole_flag)
                                {
                                    m_state = P_MATRIX_CALCULATE;
                                    break;
                                }
                                else
                                {
                                    if (dirdata[37] <= Ly_fastest)
                                    {
                                        dirdata[37] += 500; 
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                        IMUSlope();
                                        traverse();
                                        ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest, IMU_continuous);
                                        tool->Delay(100);
                                    }
                                }
                            }
                            if (hole_flag)
                            {
                                if (dirdata[34] <= Ry_fastest)
                                {
                                    dirdata[34] += 500; 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    IMUSlope();
                                    traverse();
                                    ros_com->sendContinuousValue(dirdata[33], dirdata[34], 0, dirdata[35] + continous_angle_offest, IMU_continuous);
                                    tool->Delay(200);
                                }
                                dirdata[34] = Ry_fastest;
                                m_state = P_MATRIX_CALCULATE;
                                break;
                            }
                            else
                            {
                                walking_state_string = "j%100 continousValue_Ry";
                                if (dirdata[34] >= Ry_fastest)
                                {
                                    dirdata[34] = -1500; 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    IMUSlope();
                                    traverse();
                                    ros_com->sendContinuousValue(dirdata[33], dirdata[34], 0, dirdata[35] + continous_angle_offest, IMU_continuous);
                                    tool->Delay(100);
                                }
                            }
                        }
                    }
                }
                else
                {
                    dirdata[37] = 0;
                    Turnhead_flag = false;              
                    walking_state = continuousValue_Ly; 
                    for (int j = 0;; j++)
                    {
                        if (j % 100 == 0)
                        {
                            hole_flag = true;
                            strategy_info->get_image_flag = true; 
                            ros::spinOnce();
                            tool->Delay(800);
                            for (int i = 16; i > 6; i--)
                            {
                                if (DeepMatrixValue[i] < 20)
                                {
                                    hole_flag = false;
                                    break;
                                }
                            }
                            sideline();
                            if (leftsidelinewarning == true || rightsidelinewarning == true)
                            {
                                if (hole_flag)
                                {
                                    m_state = P_MATRIX_CALCULATE;
                                    break;
                                }
                                else
                                {
                                    if (dirdata[34] <= Ry_fastest)
                                    {
                                        dirdata[34] -= 500; 
                                        strategy_info->get_image_flag = true;
                                        ros::spinOnce();
                                        IMUSlope();
                                        traverse();
                                        ros_com->sendContinuousValue(dirdata[34], dirdata[35], 0, dirdata[36] + continous_angle_offest, IMU_continuous);
                                        tool->Delay(100);
                                    }
                                }
                            }
                            if (hole_flag)
                            {

                                /////ROS_INFO("slow down");
                                if (dirdata[37] >= Ly_fastest)
                                {
                                    dirdata[37] -= 500; 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    IMUSlope();
                                    traverse();
                                    ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest, IMU_continuous);
                                    tool->Delay(200);
                                }
                                dirdata[37] = Ly_fastest;
                                m_state = P_MATRIX_CALCULATE;
                                break;
                            }
                            else
                            {
                                walking_state_string = "j%100 continousValue_Ly";
                                if (dirdata[37] <= Ly_fastest)
                                {
                                    dirdata[37] = 1500; 
                                    strategy_info->get_image_flag = true;
                                    ros::spinOnce();
                                    IMUSlope();
                                    traverse();
                                    ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest, IMU_continuous);
                                    tool->Delay(100);
                                }
                            }
                        }
                    }
                }
                break;
            }
            printinfo();
            break;   
        case P_DOOR: //紅門策略
            printinfo();
            m_state_string = "P_DOOR";
            Red_Door_flag = false;
            Blue_obs_flag = false;
            if (strategy_info->color_mask_subject_cnts[5] == 0)
            {
                first_enter_door = true;
                m_state = P_MATRIX_CALCULATE;
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1603, 100);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1603, 100);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1603, 100);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 100);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 100);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 100);
                tool->Delay(500);
                Continuous_flag = true;
                break;
            }
            //////////////////////////////stand at the center of door//////////////////////////////

            SlopeCalculate();
            in_reddoor_flag = true;
            if (face_to_door == false)
            {
                strategy_info->get_image_flag = true;
                ros::spinOnce();
                facetodoorfun();
                break;
            }
            else
            {
                first_enter_door = false;
                for (int i = 0; i < strategy_info->color_mask_subject_cnts[5]; i++)
                {
                    printinfo();
                    First_flag = false;
                    Center_door = true;
                    if (strategy_info->color_mask_subject[5][i].size > 300)
                    {
                        Red_Door_flag = true;
                        printinfo();

                        if (strategy_info->color_mask_subject[5][i].XMin > 1)
                        {
                            SlopeCalculate();
                            angle_offset = atan(slope_avg) * 180 / pi; //旋轉補償量
                            m_state = P_WALKINGGAIT;
                            walking_state = RMOVE_DOOR;
                            p_door = true;
                            break;
                        }
                        else
                        {
                            if (strategy_info->color_mask_subject[5][i].XMax < 318)
                            {
                                SlopeCalculate();
                                angle_offset = atan(slope_avg) * 180 / pi;
                                m_state = P_WALKINGGAIT;
                                walking_state = LMOVE_DOOR;
                                p_door = true;
                                break;
                            }
                        }
                        printinfo();
                        for (int j = 0; j < strategy_info->color_mask_subject_cnts[2]; j++)
                        {
                            strategy_info->get_image_flag = true;
                            ros::spinOnce();
                            tool->Delay(50);
                            for (int i = 29; i > 2; i--)
                            {
                                if (DeepMatrixValue[i] < 20)
                                {
                                    Center_door = false;
                                    break;
                                }
                            }
                            if (Center_door)
                            {
                                SlopeCalculate();
                                m_state = P_CRAWL;
                                break;
                            }
                            if (strategy_info->color_mask_subject[2][j].size > 500)
                            {
                                Blue_obs_flag = true;
                                if (First_flag)
                                {

                                    if (strategy_info->color_mask_subject[2][j].XMax > 160)
                                    {
                                        if ((strategy_info->color_mask_subject[2][j].Width - First_width) > 20)
                                        {
                                            twentyflag = false;
                                            SlopeCalculate();
                                            angle_offset = atan(slope_avg) * 180 / pi;
                                            m_state = P_WALKINGGAIT;
                                            walking_state = LMOVE_DOOR; 
                                            p_door = true;
                                        }
                                        else if ((strategy_info->color_mask_subject[2][j].Width - First_width) < -20)
                                        {
                                            SlopeCalculate();
                                            angle_offset = atan(slope_avg) * 180 / pi;
                                            m_state = P_WALKINGGAIT;
                                            walking_state = RMOVE_DOOR; 
                                            p_door = true;
                                        }
                                        else
                                        {
                                            twentyflag = true;
                                            SlopeCalculate();
                                            m_state = P_CRAWL;
                                            ///ROS_INFO("<20");
                                        }
                                    }
                                    else
                                    {
                                        if ((strategy_info->color_mask_subject[2][j].Width - First_width) > 20)
                                        {
                                            twentyflag = false;
                                            SlopeCalculate();
                                            angle_offset = atan(slope_avg) * 180 / pi;
                                            m_state = P_WALKINGGAIT;
                                            walking_state = RMOVE_DOOR;
                                            p_door = true;
                                        }
                                        else if ((strategy_info->color_mask_subject[2][j].Width - First_width) < -20)
                                        {
                                            SlopeCalculate();
                                            angle_offset = atan(slope_avg) * 180 / pi;
                                            m_state = P_WALKINGGAIT;
                                            walking_state = LMOVE_DOOR;
                                            p_door = true;
                                        }
                                        else
                                        {
                                            twentyflag = true;
                                            SlopeCalculate();
                                            m_state = P_CRAWL;
                                            ///ROS_INFO("<20");
                                        }
                                    }
                                    break;
                                }
                                else
                                {
                                    
                                    if (strategy_info->color_mask_subject[2][j].X > 159)
                                    {
                                        
                                        SlopeCalculate();
                                        angle_offset = atan(slope_avg) * 180 / pi;
                                        m_state = P_WALKINGGAIT;
                                        walking_state = LMOVE_DOOR;
                                        p_door = true;
                                    }
                                    else
                                    {
                                        SlopeCalculate();
                                        angle_offset = atan(slope_avg) * 180 / pi;
                                        m_state = P_WALKINGGAIT;
                                        walking_state = RMOVE_DOOR;
                                        p_door = true;
                                    }
                                    First_width = strategy_info->color_mask_subject[2][j].Width;
                                    First_flag = true;
                                }
                            }
                            if (!Blue_obs_flag)
                            {
                                SlopeCalculate();
                                IMUSlope();
                                m_state = P_CRAWL;
                                break;
                            }
                            if (Center_door)
                            {
                                SlopeCalculate();
                                m_state = P_CRAWL;
                                break;
                            }
                        }
                        if (!Center_door)
                        {
                            break;
                        }
                    }
                }
            }
            if (!Red_Door_flag)
            {
                first_enter_door = true;
                m_state = P_MATRIX_CALCULATE;
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1603, 100);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1603, 100);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, 1603, 100);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 100);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 100);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::HorizontalID, 2047, 100);
                tool->Delay(500);
                Continuous_flag = true;
            }
            strategy_info->get_image_flag = true;
            ros::spinOnce();
            printinfo();
            break;
        case P_CRAWL: //爬行策略
            p_door = false;
            printinfo();
            m_state_string = "P_CRAWL";
            if (face_to_door == false)
            {
                strategy_info->get_image_flag = true;
                ros::spinOnce();
                facetodoorfun();
                break;
            }
            else if (face_to_door == true && abs(slope_avg) <= 0.05) 
            {
                ros_com->sendContinuousValue(dirdata[30], dirdata[31], 0, dirdata[32], IMU_continuous);
                tool->Delay(100);
                printinfo();
                tool->Delay(3000);
                ros_com->sendBodyAuto(0, 0, 0, 0, WalkingMode::ContinuousStep, IMU_continuous);
                tool->Delay(1000);
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, 2500, 600);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, 2500, 600);
                tool->Delay(100);
                ros_com->sendHeadMotor(HeadMotorID::VerticalID, 2500, 600);
                Continuous_flag = false;
                ros_com->sendBodySector(5);
                tool->Delay(1000);
                ros_com->sendBodySector(6);
                tool->Delay(20000);
                for (int multisingleSTEP = 0; multisingleSTEP <= 10; multisingleSTEP++)
                {
                    //ROS_INFO("First crw");
                    //ros_com->sendBodySector(7);
                    tool->Delay(800);
                }
                for (int multisingleSTEP = 0; multisingleSTEP <= 20; multisingleSTEP++)
                {
                    //ROS_INFO("Second crw");
                    strategy_info->get_image_flag = true;
                    ros::spinOnce();
                    for (int i = 0; i < strategy_info->color_mask_subject_cnts[2]; i++)
                    {
                        if (strategy_info->color_mask_subject[2][i].size > 32000)
                        {
                            //ROS_INFO("stand up1");
                            crw_up = true;
                            break;
                        }
                    }
                    strategy_info->get_image_flag = true;
                    ros::spinOnce();
                    for (int i = 0; i < strategy_info->color_mask_subject_cnts[1]; i++)
                    {
                        if (strategy_info->color_mask_subject[1][i].size > 35000)
                        {
                            //ROS_INFO("stand up2");
                            crw_up = true;
                            break;
                        }
                    }
                    if (crw_up == true)
                    {
                        break;
                    }
                    //ros_com->sendBodySector(7);
                    tool->Delay(800);
                }
                tool->Delay(500);
                //ros_com->sendBodySector(8);
                tool->Delay(11000);
                ros_com->sendBodySector(29);
                continuousValue_x = 0;
                tool->Delay(6000);
                m_state = P_INIT;
            }
            printinfo();
            break;
        case P_WALKINGGAIT: //步態參數及補償量
            printinfo();
            m_state_string = "P_WALKINGGAIT";
            switch (walking_state)
            {
           
            case continuousValue_Ry:
                walking_state_string = "continousValue_Ry";   
                dirdata[34] = Ry_fastest;
                IMUSlope();
                traverse();
                ros_com->sendContinuousValue(dirdata[33], dirdata[34], 0, dirdata[35] + continous_angle_offest, IMU_continuous);
                strategy_info->get_image_flag = true;
                ros::spinOnce();
                sideline();
                continuousValue_x = 0;
                tool->Delay(100);
                break;
            case continuousValue_Ly:
                walking_state_string = "continousValue_Ly";
                dirdata[37] = Ly_fastest;
                IMUSlope();
                traverse();
                ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest, IMU_continuous);
                strategy_info->get_image_flag = true;
                ros::spinOnce();
                sideline();
                continuousValue_x = 0;
                tool->Delay(100);
                break;
            case DIRmap_RIGHT:
                walking_state_string = "DIRmap_RIGHT";
                ros_com->sendContinuousValue(dirdata[33], dirdata[34], 0, dirdata[35] + continous_angle_offest, IMU_continuous);
                tool->Delay(dirmap[2]);
                first_act_flag = false;
                break;
            case DIRmap_LEFT:
                walking_state_string = "DIRmap_LEFT";
                ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest, IMU_continuous);
                tool->Delay(dirmap[2]);
                first_act_flag = false;
                break;

            case continuousValue_Rt:
                walking_state_string = "continousValue_Rt";
                SlopeCalculate();
                ros_com->sendContinuousValue(dirdata[30], dirdata[31], 0, dirdata[32] - 6, IMU_continuous);
                strategy_info->get_image_flag = true;
                ros::spinOnce();
                continuousValue_x = 0;
                tool->Delay(300);
                break;
            case continuousValue_Lt:
                walking_state_string = "continousValue_Lt";
                SlopeCalculate();
                ros_com->sendContinuousValue(dirdata[30], dirdata[31], 0, dirdata[32] + 6, IMU_continuous);
                strategy_info->get_image_flag = true;
                ros::spinOnce();
                continuousValue_x = 0;
                tool->Delay(300);
                break;
            case continuousValue_R2t:
                walking_state_string = "continousValue_R2t";
                SlopeCalculate();
                ros_com->sendContinuousValue(dirdata[30], dirdata[31], 0, dirdata[32] - 7, IMU_continuous);
                strategy_info->get_image_flag = true;
                ros::spinOnce();
                continuousValue_x = 0;
                tool->Delay(300);
                break;
            case continuousValue_L2t:
                walking_state_string = "continousValue_L2t";
                SlopeCalculate();
                ros_com->sendContinuousValue(dirdata[30], dirdata[31], 0, dirdata[32] + 7, IMU_continuous);
                strategy_info->get_image_flag = true;
                ros::spinOnce();
                continuousValue_x = 0;
                tool->Delay(300);
                break;
            case RMOVE_DOOR:
                walking_state_string = "RMove_DOOR";
                if (dirdata[34] > Ry_fastest)
                {
                    dirdata[34] -= 100;
                    tool->Delay(100);
                }
                SlopeCalculate();
                give_angle();
                ros_com->sendContinuousValue(dirdata[33], dirdata[34], 0, dirdata[35] + continous_angle_offest, IMU_continuous);
                strategy_info->get_image_flag = true;
                ros::spinOnce();
                continuousValue_x = 0;
                tool->Delay(3000);
                break;
            case LMOVE_DOOR:
                walking_state_string = "LMove_DOOR";
                if (dirdata[37] < Ly_fastest)
                {
                    dirdata[37] += 100;
                    tool->Delay(100);
                }
                SlopeCalculate();
                give_angle();
                ros_com->sendContinuousValue(dirdata[36], dirdata[37], 0, dirdata[38] + continous_angle_offest, IMU_continuous);
                strategy_info->get_image_flag = true;
                ros::spinOnce();
                continuousValue_x = 0;
                tool->Delay(3000);
                break;
            }

            if (pcrawl_flag == true)
            {
                m_state = P_CRAWL;
                pcrawl_flag = false;
                strategy_info->get_image_flag = true;
                ros::spinOnce();
                break;
            }
            else if (pcrawl_flag == false && in_reddoor_flag == true)
            {
                m_state = P_DOOR;
                pcrawl_flag = false;
                strategy_info->get_image_flag = true;
                ros::spinOnce();
                break;
            }
            else if (first_move_flag == true)
            {
                m_state = P_INIT;
                first_cnt++;
                ros_com->sendContinuousValue(dirdata[30], dirdata[31], 0, dirdata[32], IMU_continuous);
                tool->Delay(500);
                break;
            }
            else if (p_door == true)
            {
                m_state = P_DOOR;
                p_door = false;
                strategy_info->get_image_flag = true;
                ros::spinOnce();
                break;
            }
            else
            {
                Turnhead_flag = false;
                m_state = P_MATRIX_CALCULATE;
                break;
            }
            break;
        }
    }
    else //策略指撥關閉
    {
        if (stand_flag == true)
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
            tool->Delay(2000);
            ros_com->sendBodySector(29);
            tool->Delay(2000);
            first_cnt = 0;
            ROS_INFO("stop");
        }
        m_state = P_INIT;
        readwalkinggait();
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
        if (5 < strategy_info->getIMUValue().Yaw && strategy_info->getIMUValue().Yaw <= 180)
        {
            IMU_slope = strategy_info->getIMUValue().Yaw;
        }
        else if (-5 > strategy_info->getIMUValue().Yaw && strategy_info->getIMUValue().Yaw >= -179)
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
        if (abs(IMU_slope) >= 41)
        {
            continous_angle_offest = 9;
        }
        else if (abs(IMU_slope) >= 28 && abs(IMU_slope) < 41)
        {
            continous_angle_offest = 6;
        }
        else if (abs(IMU_slope) > 4 && abs(IMU_slope) < 28)
        {
            continous_angle_offest = 4;
        }
        else
        {
            continous_angle_offest = 0;
        }
    }
    else
    {
        if (abs(IMU_slope) >= 41)
        {
            continous_angle_offest = -8;
        }
        else if (abs(IMU_slope) >= 28 && abs(IMU_slope) < 41)
        {
            continous_angle_offest = -5;
        }
        else if (abs(IMU_slope) > 4 && abs(IMU_slope) < 28)
        {
            continous_angle_offest = -3;
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
            continous_angle_offest = 2;
        }
        else if (abs(IMU_slope) > 10 && abs(IMU_slope) <= 15)
        {
            continous_angle_offest = 1;
        }
        else if (abs(IMU_slope) > 5 && abs(IMU_slope) <= 10)
        {
            continous_angle_offest = 0;
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
            continous_angle_offest = -2;
        }
        else if (abs(IMU_slope) > 10 && abs(IMU_slope) <= 15)
        {
            continous_angle_offest = -1;
        }
        else if (abs(IMU_slope) > 5 && abs(IMU_slope) <= 10)
        {
            continous_angle_offest = 0;
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
        dirdata[39] = tool->readvalue(fin, "IMU_continuous", 0);
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
    for (int i = 0; i < 32; i++)
    {
        DeepMatrixValue[i] = msg.DeepMatrix[i];
    }
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
        if (strategy_info->color_mask_subject[1][i].size > 35 && strategy_info->color_mask_subject[1][i].YMax > 230)
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
    ROS_INFO("\n\n\n");
    ROS_INFO("%s", m_state_string.c_str());
    if (zero_flag)
    {
        //walking_state_string="forward";
        ROS_INFO("zero_flag = true");
    }
    else
        ROS_INFO("zero_flag = false");
    ROS_INFO("%s",walking_state_string.c_str());
    ROS_INFO("continuousValue_x = %5d", continuousValue_x);
    ROS_INFO("[30] = %5d [31] = %5d [32] = %5d", dirdata[30], dirdata[31], dirdata[32]);
    ROS_INFO("[33] = %5d [34] = %5d [35] = %5d", dirdata[33], dirdata[34], dirdata[35]);
    ROS_INFO("[36] = %5d [37] = %5d [38] = %5d", dirdata[36], dirdata[37], dirdata[38]);
    if (leftsidelinewarning)
        ROS_INFO("leftsidelinewarning = true");
    else
        ROS_INFO("leftsidelinewarning = false");
    if (rightsidelinewarning)
        ROS_INFO("rightsidelinewarning = true");
    else
        ROS_INFO("rightsidelinewarning = false");
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
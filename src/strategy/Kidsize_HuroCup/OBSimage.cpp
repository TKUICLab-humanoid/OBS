#include "strategy/OBSimage.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "OBSimage");
	ros::NodeHandle nh;
	OBSimage OBSimage(nh);

	ros::Rate loop_rate(30);
	

	while (nh.ok()) 
	{
		OBSimage.strategymain();
		ros::spinOnce();
		loop_rate.sleep();
        
	}
	return 0;
}

void OBSimage::strategymain()
{  
	if(strategy_info->getStrategyStart())
	{	
		cv::Mat image = strategy_info->cvimg->image;

		ROS_INFO("Deep Matrix");
			
        for(int compress_width = 0 ; compress_width < IMAGEWIDTH/10  ; compress_width++)
        {
            Deep_Matrix[compress_width] = 0;
            for(int compress_height = IMAGEHEIGHT/10 - 1 ; compress_height > -1 ; compress_height--)
            {
                bValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 0));
                gValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 1));
                rValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 2));

                if((*rValue == 128 && *gValue == 0 && *bValue == 128) || (*rValue == 0 && *gValue == 128 && *bValue == 128))
                {
					Deep_Matrix[compress_width] = (IMAGEHEIGHT/10 - 1) - compress_height;
					break;
                }
				if(compress_height == 0)
				{
					Deep_Matrix[compress_width] = 24;
				}
            }

			printf("%2d,",Deep_Matrix[compress_width]);
        }
		printf("\n");
		/*for(int compress_height = 0 ; compress_height < IMAGEHEIGHT/10  ; compress_height++)
        {
            for(int compress_width = 0 ; compress_width < IMAGEWIDTH/10 ; compress_width++)
            {
                bValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 0));
                gValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 1));
                rValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 2));
				if((*rValue == 128 && *gValue == 0 && *bValue == 128) || (*rValue == 0 && *gValue == 128 && *bValue == 128))
				{
					printf("1  ");
				}
				else
				{
					printf("0  ");
				}
			}
			printf("\n");
		}*/

		INIT_parameter();

		//0905++++
	////////////////////有進紅門，紅門case內影像判斷////////////////////

		if (strategy_info->color_mask_subject_cnts[5] != 0)				
		{
			for(int i = 0; i < strategy_info->color_mask_subject_cnts[5]; i++)
			{
				if(strategy_info->color_mask_subject[5][i].size > 3500)
				{
					//ROS_INFO("red area = %d ",strategy_info->color_mask_subject[5][i].size);
					ROS_INFO("IN_RED");
					in_reddoor_flag = true;
					b_obs_flag = false;
					y_obs_flag = false;
					//ROS_INFO("in_reddoor_flag = true");


					SlopeCalculate();
					//再去策略端拿slope_avg做旋轉修正


					if (abs(slope_avg) <= 0.7)
					{
						for (int i = 0; i < strategy_info->color_mask_subject_cnts[5]; i++)
						{
							LD = 319 - strategy_info->color_mask_subject[5][i].XMin;
							RD = strategy_info->color_mask_subject[5][i].XMax - 0;
						}
					}
					
					for(int i = 0; i < strategy_info->color_mask_subject_cnts[2]; i++)	//單塊藍色判斷
					{
						L_XMAX = strategy_info->color_mask_subject[2][i].XMax - 0;
						R_XMIN = 319 - strategy_info->color_mask_subject[2][i].XMin;
					}


					for(int i = 0; i < strategy_info->color_mask_subject_cnts[2]; i++)   //兩塊藍色同時再螢幕內
					{
						if(strategy_info->color_mask_subject[2][i].size > 5000 )
						{
							XMax_one = strategy_info->color_mask_subject[2][0].XMax;
							XMin_one = strategy_info->color_mask_subject[2][0].XMin;
							XMin_two = strategy_info->color_mask_subject[2][1].XMin;
							XMax_two = strategy_info->color_mask_subject[2][1].XMax;
						}
					}


					if(XMin_one > XMin_two)						//比較所有xmax &&　xmin ,確保抓到的是正確的值
					{
						RightblueOBS_XMin = XMin_one;
					}
					else if (XMin_one < XMin_two)
					{
						RightblueOBS_XMin = XMin_two;
					}
					if(XMax_one > XMax_two)
					{
						LeftblueOBS_XMax = XMax_two;
					}
					else if(XMax_one < XMax_two)
					{
						LeftblueOBS_XMax = XMax_one;
					}
					//ROS_INFO("XMin_one = %d",XMin_one);
					//ROS_INFO("XMin_two = %d",XMin_two);
					//ROS_INFO("XMax_two = %d",XMax_two);
					//ROS_INFO("XMax_one = %d",XMax_one);
					ros::spinOnce();
                    tool->Delay(50);
				}
				else
				{
					in_reddoor_flag = true;
					b_obs_flag = false;
					y_obs_flag = false;
					ROS_INFO("RED is not big enough");
					ros::spinOnce();
                    tool->Delay(50);
				}
				
			}
		}
		////////////////////未進紅門，其餘影像判斷////////////////////		
		else
		{
			ROS_INFO("NOT_IN_RED");
			in_reddoor_flag = false;
			ROS_INFO("in_reddoor_flag = false");
			//if(Filter_Matrix[i] > 0)
			//{
				if ((strategy_info->color_mask_subject_cnts[2] != 0) && (strategy_info->color_mask_subject_cnts[1] != 0))
				{
					ROS_INFO("obs are b & y");
					b_obs_flag = true;
					y_obs_flag = true;
					ROS_INFO("b_obs_flag = true y_obs_flag = true");
				}
				else if (strategy_info->color_mask_subject_cnts[2] != 0)
				{
					ROS_INFO("obs is b");
					b_obs_flag = true;
					y_obs_flag = false;
					ROS_INFO("b_obs_flag = true y_obs_flag = false");
				}
				else if (strategy_info->color_mask_subject_cnts[1] != 0)
				{
					ROS_INFO("obs is y");
					b_obs_flag = false;
					y_obs_flag = true;	
					ROS_INFO("b_obs_flag = false y_obs_flag = true");
				}
				else 
				{
					ROS_INFO("no obs");
					b_obs_flag = false;
					y_obs_flag = false;
					ROS_INFO("b_obs_flag = false y_obs_flag = false");
				}
				ros::spinOnce();
            	tool->Delay(50);
			//}

		//0905++++

			ROS_INFO("Focus Matrix");
			for(int i = 0; i < 32 ;i++)
			{
			printf("%2d,",Focus_Matrix[i]);
			}
			printf("\n");
			ROS_INFO("Filter Matrix");
			ros::spinOnce();
            tool->Delay(50);

			for(int i = 0; i < 32 ;i++)
			{
				Filter_Matrix[i] = Focus_Matrix[i] - Deep_Matrix[i];

				if(Filter_Matrix[i] > 0)	//obstacle in focus matrix
				{
					Xc_count++;
					Xi_sum += i;
					Xc = (float)Xi_sum /(float) Xc_count;			//get x_avg when obstacle is in focus matrix 
					//printf("Xi_sum = %d,Xc_count = %d,Xc  = %.3lf \n",Xi_sum,Xc_count,Xc);
				}
				else												
				{
					Filter_Matrix[i] = 0;
				}
				//printf("%2d,",Filter_Matrix[i]);

				if(Deep_Matrix[i] < Dy)							//get DeepMatrix min
				{
					Dy = Deep_Matrix[i];
				}

				printf("%2d,",Filter_Matrix[i]);
				//calculate WR WL
				//WR += (32-i) * Filter_Matrix[i];
				//WL += (i+1) * Filter_Matrix[i];
				WR += (32-i) * (24-Deep_Matrix[i]);
				WL += (i+1) * (24-Deep_Matrix[i]);


			}
			ros::spinOnce();
            tool->Delay(50);
			printf("\n");

			/*if(abs(WR-WL) < 5 && (WR > 0) && (WL > 0))		//WR = WL
			{
				for(int i = 0 ; i < 32 ; i++) 
				{
					if(i < 16)
					{
						W_L += Deep_Matrix[i];
					}
					else   //32 > i > 16 
					{
						W_R += Deep_Matrix[i];
					}
				}

				if(W_L > W_R)
				{
					WL += 100;
				}
				else
				{
					WR += 100;
				}
			}*/

			if((WR - WL) > 5)											
			{
				Xb = 0;
				ROS_INFO("Obstacle in left");
			}
			else if((WL - WR) > 5)
			{
				Xb = 31;
				ROS_INFO("Obstacle in right");
			}
			ros::spinOnce();
            tool->Delay(50);

			Dx = Xc - Xb;
		}

		//0905++++
		strategy_info->get_image_flag = true;
        ros::spinOnce();
		tool->Delay(150);
		//0905++++

		ROS_INFO("W_R = %d,W_L = %d",W_R,W_L);
		ROS_INFO("Xb = %.3lf, Dx = %.3lf",Xb,Dx);
		ROS_INFO("Xc_count = %d, Xi_sum = %d, Xc = %.3lf",Xc_count,Xi_sum,Xc);
		ROS_INFO("Dy = %d, WR = %d, WL = %d",Dy,WR,WL);

		getparameter_parameter.Dy = Dy;
		getparameter_parameter.Dx = Dx;

		//0905++++
		ROS_INFO("L_XMAX = %3d",L_XMAX);
		ROS_INFO("R_XMIN = %3d",R_XMIN);
		printf("\n");
		//getparameter_parameter.Xc = Xc;
		getparameter_parameter.RD = RD;
		getparameter_parameter.LD = LD;
		getparameter_parameter.slope_avg = slope_avg;
		getparameter_parameter.LeftblueOBS_XMax = LeftblueOBS_XMax;
		getparameter_parameter.RightblueOBS_XMin = RightblueOBS_XMin;
		getparameter_parameter.in_reddoor_flag = in_reddoor_flag;
		getparameter_parameter.b_obs_flag = b_obs_flag;
		getparameter_parameter.y_obs_flag = y_obs_flag;
		getparameter_parameter.L_XMAX = L_XMAX;
		getparameter_parameter.R_XMIN = R_XMIN;
		//0905++++



		GetParameter_Publish.publish(getparameter_parameter);

        ROS_INFO("\n");
		//cv::imshow("image",image);
		cv::waitKey(1);
    }
}

void OBSimage::INIT_parameter()
{
	Xc_count = 0;
	Xi_sum = 0;
	Xc = 0;
	W_R = 0;
	W_L = 0;
	WR = 0;
	WL = 0;	
	Dy = Deep_Matrix[0];
}
void OBSimage::SlopeCalculate()			//計算斜率之副函式
{
    bool Check_label_model_flag = true;
    int slope_rand[4];
    int slope_Y[4];
    float slope[3];
    slope_avg = 1000000.0;

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
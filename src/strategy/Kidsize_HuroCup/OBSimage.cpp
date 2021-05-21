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

		ROS_INFO("Focus Matrix");
		for(int i = 0; i < 32 ;i++)
        {
           printf("%2d,",Focus_Matrix[i]);
        }
		printf("\n");
		ROS_INFO("Filter Matrix");

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
			WR += (32-i) * Filter_Matrix[i];
			WL += (i+1) * Filter_Matrix[i];

		}
		printf("\n");
		if(WL < WR)											
		{
			Xb = 0;
			ROS_INFO("Obstacle in left");
		}
		else if(WR < WL)
		{
			Xb = 31;
			ROS_INFO("Obstacle in right");
		}
		else if((WR == WL) && (WR > 0) && (WL > 0))		//WR = WL
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
				WL += 10;
			}
			else
			{
				WR += 10;
			}
		}

		Dx = Xc - Xb;

		ROS_INFO("W_R = %d,W_L = %d",W_R,W_L);
		ROS_INFO("Xb = %.3lf, Dx = %.3lf",Xb,Dx);
		ROS_INFO("Xc_count = %d, Xi_sum = %d, Xc = %.3lf",Xc_count,Xi_sum,Xc);
		ROS_INFO("Dy = %d, WR = %d, WL = %d",Dy,WR,WL);

		deepmatrix_parameter.Dy = Dy;
		deepmatrix_parameter.Dx = Dx;

		DeepMatrix_Publish.publish(deepmatrix_parameter);

        ROS_INFO("\n");
////////////////////////////////////opencv/////////////////////////////////////////////
         cv::resize(image, publish_image, cv::Size(320, 240),CV_INTER_LINEAR);
		cv::waitKey(1);
        msg_compressimage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", publish_image).toImageMsg();
        pub_colormodel.publish(msg_compressimage);
        image.release();
////////////////////////////////////opencv////////////////////////////////////////////
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
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
            for(int compress_width = 0 ; compress_width < IMAGEWIDTH/10  ; compress_width++)
            {
                DeepMatrix_cnt[compress_width] = 0;
                for(int compress_height = IMAGEHEIGHT/10 - 1 ; compress_height > -1 ; compress_height--)
                {
                    bValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 0));
                    gValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 1));
                    rValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 2));

                    if((*rValue == 128 && *gValue == 0 && *bValue == 128) || (*rValue == 0 && *gValue == 128 && *bValue == 128))
                    {
                        //DeepMatrix_cnt[compress_width]++;
						if((compress_height - 1) > -1)
						{
							bValue = (image.data + (((compress_height - 1)*IMAGEWIDTH/10 + compress_width) * 3 + 0));
                        	gValue = (image.data + (((compress_height - 1)*IMAGEWIDTH/10 + compress_width) * 3 + 1));
                        	rValue = (image.data + (((compress_height - 1)*IMAGEWIDTH/10 + compress_width) * 3 + 2));
                        	if((*rValue == 128 && *gValue == 0 && *bValue == 128) || (*rValue == 0 && *gValue == 128 && *bValue == 128))
                        	{
                            //DeepMatrix_cnt[compress_width]++;
								if((compress_height - 2) > -1)
								{
									bValue = (image.data + (((compress_height - 2)*IMAGEWIDTH/10 + compress_width) * 3 + 0));
                        			gValue = (image.data + (((compress_height - 2)*IMAGEWIDTH/10 + compress_width) * 3 + 1));
                       				rValue = (image.data + (((compress_height - 2)*IMAGEWIDTH/10 + compress_width) * 3 + 2));
                        			if((*rValue == 128 && *gValue == 0 && *bValue == 128) || (*rValue == 0 && *gValue == 128 && *bValue == 128))
                        			{
                            			DeepMatrix_cnt[compress_width] = (IMAGEHEIGHT/10 - 1) - compress_height;
										break;
                        			}
								}
								else
								{
									DeepMatrix_cnt[compress_width] = 22;
									break;
								}
                        	}
						}
						else
						{
							DeepMatrix_cnt[compress_width] = 23;
							break;
						}
                    }
					if(compress_height == 0)
					{
						DeepMatrix_cnt[compress_width] = 24;
					}
                }
            }

			for(int compress_height = 0 ; compress_height < IMAGEHEIGHT/10  ; compress_height++)
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
			}

			printf("\nDeep Matrix\n");
            for(int i = 0; i < 32 ;i++)
            {
                deepmatrix.DeepMatrix.push_back(DeepMatrix_cnt[i]);
                printf("%2d,",DeepMatrix_cnt[i]);
            }
			printf("\nFocus Matrix\n");
			for(int i = 0; i < 32 ;i++)
            {
                printf("%2d,",Focus_matrix[i]);
            }

			printf("\nFilter Matrix\n");

			for(int i = 0; i < 32 ;i++)
			{
				FilterMatrix[i] = Focus_matrix[i] - DeepMatrix_cnt[i]; 
				if(FilterMatrix[i] < 0)
				{
					FilterMatrix[i] = 0;
				}
				else								//obstacle in focus matrix
				{
					Xc_count++;
					Xi_sum += DeepMatrix_cnt[i];
					Xc = Xi_sum / Xc_count;			//get x_avg when obstacle is in focus matrix 
				}
				printf("%2d,",FilterMatrix[i]);

				Dy = DeepMatrix_cnt[0];
				if(DeepMatrix_cnt[i] < Dy)			//get DeepMatrix min
				{
					Dy = DeepMatrix_cnt[i];
				}
			}
			WR = 0;
			WL = 0;
			for(int i = 1; i < 33 ;i++)
			{
				WR += (33-i) * FilterMatrix[i-1];
				WL += i * FilterMatrix[i-1];
			}

			if(WR > WL)
			{
				Xb = 0;
				Dx = Xc - Xb;
			}
			else //WR<WL
			{
				Xb = 31;
				Dx = Xc - Xb;
			}

			printf("Xb = %d, Dx = %lf\n",Xb,Dx);
			printf("Xc_count = %d, Xi_sum = %lf, Xc = %lf\n",Xc_count,Xi_sum,Xc);
			printf("Dy = %d\n",Dy);
			printf("\nWR = %d,WL = %d\n",WR,WL);

            //DeepMatrix_Publish.publish(deepmatrix);
            deepmatrix.DeepMatrix.clear();
            printf("\n");
////////////////////////////////////opencv/////////////////////////////////////////////
            cv::resize(image, publish_image, cv::Size(320, 240),CV_INTER_LINEAR);
			//cv::imshow("publish_image",publish_image);
			cv::waitKey(1);
            msg_compressimage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", publish_image).toImageMsg();
            pub_colormodel.publish(msg_compressimage);
            image.release();
////////////////////////////////////opencv////////////////////////////////////////////
        }
    
	else
	{
		
	}
}

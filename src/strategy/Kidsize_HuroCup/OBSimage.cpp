#include "strategy/OBSimage.h"

int totalL = 0;
int totalR = 0;
int Left[32]= {1,1,1,1,1,1,1,1,2,2,2,2,2,2,2,2,3,3,3,3,3,3,3,3,4,4,4,4,4,4,4,4};
int Right[32]={4,4,4,4,4,4,4,4,3,3,3,3,3,3,3,3,2,2,2,2,2,2,2,2,1,1,1,1,1,1,1,1};
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
            cv::Mat compressimage(24,32,CV_8UC3);
            /*for(int compress_height = 0 ; compress_height < IMAGEHEIGHT/10 ; compress_height++)
            {
                for(int compress_width = 0 ; compress_width < IMAGEWIDTH/10 ; compress_width++)
                {
                    bValue = (compressimage.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 0));
                    gValue = (compressimage.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 1));
                    rValue = (compressimage.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 2));
                    for(int lheight = 0 ; lheight < 10 ; lheight++)
                    {
                        for(int lwidth = 0 ; lwidth < 10 ; lwidth++)
                        {
						ROS_INFO("%d",strategyinfo->Label_Model[(lheight+compress_height*10)*IMAGEWIDTH+(lwidth+compress_width*10)]);
                            if(strategyinfo->Label_Model[(lheight+compress_height*10)*IMAGEWIDTH+(lwidth+compress_width*10)] == BlueLabel)
                            {
                                color_cnt++;                            
                            }
                            else if(strategyinfo->Label_Model[(lheight+compress_height*10)*IMAGEWIDTH+(lwidth+compress_width*10)] == RedLabel)
                            {
                                color_cnt+=2;
                            }
                        }
                    }
					//ROS_INFO("%d",color_cnt);
                    if(color_cnt < 50)
                    {
                        *bValue = 0;
                        *gValue = 0;
                        *rValue = 0;
						//printf("0  ");
                    }
                    else if(color_cnt <= 150)
                    {
                        *bValue = 128;
                        *gValue = 0;
                        *rValue = 128;
						//printf("1  ");
                    }
                    else
                    {
                        *bValue = 255;
                        *gValue = 255;
                        *rValue = 0;
                    }
                    color_cnt = 0;
                }
				//printf("\n");
            }
			cv::resize(compressimage, publish_image, cv::Size(320, 240),CV_INTER_LINEAR);
			msg_compressimage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", publish_image).toImageMsg();
            pub_colormodel.publish(msg_compressimage);*/
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
			/*for(int compress_width = 0 ; compress_width < IMAGEWIDTH/10 ; compress_width++)
            {
                for(int compress_height = 0 ; compress_height < IMAGEHEIGHT/10  ; compress_height++)
                {
                    bValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 0));
                    gValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 1));
                    rValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 2));
					if(*rValue == 128 &&  *gValue == 0 && *bValue == 128)
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
            totalL = 0;
            totalR = 0;
            for(int i = 0; i < 32 ;i++)
            {
                deepmatrix.DeepMatrix.push_back(DeepMatrix_cnt[i]);
                printf("%d,",DeepMatrix_cnt[i]);
                //printf("%d,",deepmatrix.DeepMatrix[i]);
                    totalL += DeepMatrix_cnt[i]*DeepMatrix_cnt[i];
                    totalR += DeepMatrix_cnt[i]*DeepMatrix_cnt[i];
            }
            printf("totalL = %d\ntotalR = %d", totalL,totalR);
            DeepMatrix_Publish.publish(deepmatrix);
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

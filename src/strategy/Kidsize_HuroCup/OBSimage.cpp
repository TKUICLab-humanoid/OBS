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
    int FocusMatrix[32] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10}; //攝影機內之焦點矩陣
    /*int FocusMatrix_R[32] = {4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1};
    int FocusMatrix_L[32] = {1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4};*/
    int LeftMove[32]  = { 0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  1,  2,  3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 21};
    int RightMove[32] = {21, 21, 20, 19, 18, 17, 16, 15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    
	if(strategy_info->getStrategyStart())
	{	
        for (int i = 0; i < DeepMatrixSize; i++) //深度矩陣之運算    
        {
            if (FocusMatrix[i] - DeepMatrix_cnt[i] > 0)
            {                                      
                FilterMatrix[i] = FocusMatrix[i] - DeepMatrix_cnt[i]; 
                printf("%d,",FilterMatrix[i]);
                //printf("\n");
            }
            else 
            {
                FilterMatrix[i] = 0; 
                printf("%d,",FilterMatrix[i]);
                //printf("keep gogogogo");
                //printf("FilterMatrix = 0",FilterMatrix[i]);
                //printf("\n");
            }
        }
            
            //cv::Mat compressimage(24,32,CV_8UC3);

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
					/*if((*rValue == 128 && *gValue == 0 && *bValue == 128) || (*rValue == 0 && *gValue == 128 && *bValue == 128))
					{
						printf("1 ");
					}
					else
					{
						printf("0 ");
					}*/
				}
			}
            printf("\n");
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
            printf("\n");
            //printf("totalL = %d\ntotalR = %d", totalL,totalR);
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

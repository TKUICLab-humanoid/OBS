#include "strategy/OBSimage.h"
#include <time.h>

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
    clock_t start, end;
    cv::Mat mask128 (240,320,CV_8UC1, cv::Scalar(128));
    cv::Mat mask0 (240,320,CV_8UC1, cv::Scalar(0));
    std::vector<cv::Mat> rgbChannels(3);
    cv::Mat roi;
    //cv::namedWindow("opencv resize", cv::WINDOW_NORMAL);
    //cv::namedWindow("10*10", cv::WINDOW_NORMAL);

	if(strategy_info->getStrategyStart())
	{	

            cv::Mat ori_image = strategy_info->cvimg->image;
            start = clock();
            cv::Mat image;
            cv::resize(ori_image, image, cv::Size(32, 24));
            end = clock();
            ROS_INFO("opencv resize time: %f", double(end-start)/CLOCKS_PER_SEC);
            //cv::imshow("opencv resize", image);

            
            start = clock();
            cv::Mat compress_img = cv::Mat::zeros(24,32,CV_8UC1);

	        cv::split(ori_image, rgbChannels);
            cv::Mat B = rgbChannels[0];
            cv::Mat G = rgbChannels[1];
            cv::Mat R = rgbChannels[2];

            cv::Mat maskB = B == mask128;
            cv::Mat maskG = G == mask0;
            cv::Mat maskR = R == mask128;

            cv::bitwise_and(maskB, maskG, maskB);
            cv::bitwise_and(maskB, maskR, maskB);

            for(int r = 0; r < 24; r++)
            {
                for (int c = 0; c < 32; c++)
                {
                    roi = maskB(cv::Rect(c*10, r*10, 10, 10));
                    int val = cv::countNonZero(roi);
                    if(val > 20)
                    {
                        *(compress_img.data + (r*32+c)) = 255;
                    }
                }
            }

            end = clock();
            ROS_INFO("10*10 time: %f", double(end-start)/CLOCKS_PER_SEC);
            //cv::imshow("10*10", compress_img);
            //cv::waitKey(1);

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

            totalL = 0;
            totalR = 0;
            for(int i = 0; i < 32 ;i++)
            {
                deepmatrix.DeepMatrix.push_back(DeepMatrix_cnt[i]);
                //printf("%d,",DeepMatrix_cnt[i]);
                //printf("%d,",deepmatrix.DeepMatrix[i]);
                    totalL += DeepMatrix_cnt[i]*DeepMatrix_cnt[i];
                    totalR += DeepMatrix_cnt[i]*DeepMatrix_cnt[i];
            }
            //printf("totalL = %d\ntotalR = %d", totalL,totalR);
            DeepMatrix_Publish.publish(deepmatrix);
            deepmatrix.DeepMatrix.clear();
            //printf("\n");
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

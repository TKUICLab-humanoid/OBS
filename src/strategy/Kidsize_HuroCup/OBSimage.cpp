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
        cv::Mat compressimage(24,32,CV_8UC3);
        cv::Mat image = strategy_info->cvimg->image;

        for(int compress_width = 0 ; compress_width < IMAGEWIDTH/10  ; compress_width++)
        {
            //minY = 24;
            DeepMatrix_cnt[compress_width] = 0;
            for(int compress_height = IMAGEHEIGHT/10 - 1 ; compress_height > -1 ; compress_height--)
            {
                bValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 0));
                gValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 1));
                rValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 2)); 
                if((*rValue == 128 && *gValue == 0 && *bValue == 128) || (*rValue == 0 && *gValue == 128 && *bValue == 128))
                {
                    DeepMatrix_cnt[compress_width] = (IMAGEHEIGHT/10 - 1) - compress_height;
                    if(DeepMatrix_cnt[compress_width] < minY/*<= DeepMatrix_cnt[compress_width+1] || DeepMatrix_cnt[compress_width] <= DeepMatrix_cnt[compress_width-1]*/)
                    {
                        minY = DeepMatrix_cnt[compress_width];
                    }
                    else if(DeepMatrix_cnt[compress_width] > 0 && minY ==0)
                    {
                        minY = DeepMatrix_cnt[compress_width];
                    }
	    			break;
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
        for(int i = 0; i < 32 ;i++)
        {
            
            if (Focus[i] >= DeepMatrix_cnt[i])
            {   
                V[i] = Focus[i] - DeepMatrix_cnt[i];
            }
            else
            {
                V[i] = 0;
            }
        }
        printf("\n\n");
        printf("FocusMatrix:");

        for(int j =0; j < 32 ; j++)
        {
            printf("%d ,",Focus[j]);
        }
        printf("\n");
        printf("DeepMatrix :");
        
        for(int i =0; i < 32 ; i++)
        {
            printf("%d ,",DeepMatrix_cnt[i]);
            deepmatrix.DeepMatrix.push_back(DeepMatrix_cnt[i]);            
        }
        printf("\n");
        printf(" VMartrix  :");

        for(int k =0; k < 32 ; k++)
        {
            printf("%d ,",V[k]); 
        }

        printf("\n");
        printf("dy : %d",minY);
        printf("\n");
        printf("dx : %d");
        printf("\n");
        printf("xc : %d");
        printf("\n");
        printf("xb : %d");

DeepMatrix_Publish.publish(deepmatrix);
deepmatrix.DeepMatrix.clear();
printf("\n");
////////  ////////////////////////opencv/////////////////////////////////////////////
cv::resize(image, publish_image, cv::Size(320, 240),CV_INTER_LINEAR);
cv::waitKey(1);
msg_compressimage = cv_bridge::CvImage(std_msgs::Header(), "bgr8", publish_image).toImageMsg();
pub_colormodel.publish(msg_compressimage);
image.release();
////////////////////////////////////opencv////////////////////////////////////////////
    }
}

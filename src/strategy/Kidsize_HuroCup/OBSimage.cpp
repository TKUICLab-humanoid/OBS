#include "strategy/OBSimage.h"
//int minY = 24;
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
        printf("\n");
        printf("\n");
        printf("\n");
        
        cv::Mat compressimage(24,32,CV_8UC3);
        cv::Mat image = strategy_info->cvimg->image;

        for(int compress_width = 0 ; compress_width < IMAGEWIDTH/10  ; compress_width++)
        {
            DeepMatrix_cnt[compress_width] = 0;
            for(int compress_height = IMAGEHEIGHT/10 - 1 ; compress_height > -1 ; compress_height--)
            {
                minY = 24;
                bValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 0));
                gValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 1));
                rValue = (image.data + ((compress_height*IMAGEWIDTH/10 + compress_width) * 3 + 2)); 
                if((*rValue == 128 && *gValue == 0 && *bValue == 128) || (*rValue == 0 && *gValue == 128 && *bValue == 128))
                {
                    DeepMatrix_cnt[compress_width] = (IMAGEHEIGHT/10 - 1) - compress_height;
                    //if(minY > DeepMatrix_cnt[compress_width] && DeepMatrix_cnt[compress_width])
                    break;
                }
	    		if(compress_height == 0)
	    		{
	    			DeepMatrix_cnt[compress_width] = 24;
                    //break;                   
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
        //printf("dx : %d");
        for(int m = 0; m < 32;m++)
        {   
            WR += (33 - (m+1))*V[m];
            WL += (m+1)*V[m];
            if(m == 31)
            {
                printf("WR = %d ",WR);
                printf("\n");
                printf("WL = %d ",WL);
                printf("\n");
                printf("xb :");
                if(WR >= WL)
                {
                    xb = 0;
                    dx = xc - xb;
                    printf("%d",xb);
                    printf("\n");
                    printf("dx :");
                    printf("%d",dx);
                    printf("\n");
                    printf("GOGOGOGO Rright __case : walking trun right__");
                    printf("\n");
                }
                else
                {
                    xb = 319;
                    dx = xb - xc;
                    printf("%d",xb);
                    printf("\n");
                    printf("dx :");
                    printf("%d",dx);
                    printf("\n");
                    printf("GOGOGOGO Left   __case : walking trun left__");
                    printf("\n");
                }  
                WR =0;
                WL =0;
                
            }
        }
        printf("dy :");
        for(int l = 0; l < 32 ;l++)
        {
            if(l == 0)
            {
                if(DeepMatrix_cnt[l] <= minY)
                {
                    minY = DeepMatrix_cnt[l] ;
                }
            }
            else
            {
                if(  DeepMatrix_cnt[l] <= minY && DeepMatrix_cnt[l]<DeepMatrix_cnt[l-1])
                {   
                    minY = DeepMatrix_cnt[l] ; 
                }
            }
            
            if(l ==31) 
            {
                printf("%d",minY);
                printf("\n");

            }
        }
        printf("xc :");
        for(int n = 0;n < 32;n++)
        {
            if(V[n] > 0)
            {
                X  += Xi[n]; 
                xin ++;
            }
            else
            {
            }
            if (n == 31)
            {
                if(X == 0 || xin ==0)
                {
                    xc = 0;
                    printf("0");
                    X = 0;
                    xin = 0;
                }
                else
                {
                    xc = ( X/xin );
                    printf("%d",xc);
                    X = 0;
                    xin = 0;
                }
            }
        }
        printf("\n");
        printf("\n");
        printf("\n");
        printf("\n");
        printf("\n");
        printf("\n");
        printf("\n");
        printf("\n");
        printf("\n");
        printf("\n");
        printf("\n");
        printf("\n");
        //printf("\n");
        //printf("\n");

DeepMatrix_Publish.publish(deepmatrix);
deepmatrix.DeepMatrix.clear();
/*deepmatrix.
deepmatrix.
deepmatrix.
deepmatrix.
deepmatrix.
deepmatrix.
deepmatrix.*/

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

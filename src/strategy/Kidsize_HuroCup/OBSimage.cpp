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
    Xc = 0;
    Xb = 0;
    WR = 0;
    WL = 0;
    WL_sigma = 0;
    WR_sigma = 0;
    Dx = 0;
	if(strategy_info->getStrategyStart())
    {	
        //cv::Mat compressimage(24,32,CV_8UC3);

        cv::Mat image = strategy_info->cvimg->image;

        for(int compress_height = 0 ; compress_height < IMAGEHEIGHT  ; compress_height++)   //顯示24X32中有沒有障礙物
        {
            for(int compress_width = 0 ; compress_width < IMAGEWIDTH ; compress_width++)
            {
                bValue = (image.data + ((compress_height*IMAGEWIDTH + compress_width) * 3 + 0));
                gValue = (image.data + ((compress_height*IMAGEWIDTH + compress_width) * 3 + 1));
                rValue = (image.data + ((compress_height*IMAGEWIDTH + compress_width) * 3 + 2));
                /*if((*rValue == 128 && *gValue == 0 && *bValue == 128) || (*rValue == 0 && *gValue == 128 && *bValue == 128))
                {
                    printf("1 ");
                }
                else
                {
                    printf("0 ");
                }*/
            }
            printf("\n");
        }
        

//////////////////////////////////////顯示24X32中有沒有障礙物/////////////////////////////////////////////


        printf("\n"),printf("D = ");                                //計算D && Dy                             
        for(int compress_width = 0 ; compress_width < IMAGEWIDTH  ; compress_width++)   
        {
            DeepMatrix[compress_width] = 0;
            Dy = DeepMatrix[0];
            for(int compress_height = IMAGEHEIGHT - 1 ; compress_height > -1 ; compress_height--)
            {
                bValue = (image.data + ((compress_height*IMAGEWIDTH + compress_width) * 3 + 0));
                gValue = (image.data + ((compress_height*IMAGEWIDTH + compress_width) * 3 + 1));
                rValue = (image.data + ((compress_height*IMAGEWIDTH + compress_width) * 3 + 2));

                if((*rValue == 128 && *gValue == 0 && *bValue == 128) || (*rValue == 0 && *gValue == 128 && *bValue == 128))
                {
                    DeepMatrix[compress_width] = (IMAGEHEIGHT - 1) - compress_height;
                    break;
                }
                if(compress_height == 0)
                {
                    DeepMatrix[compress_width] = 24;
                }
            }
        }
        for(int i = 0; i < 32 ;i++)
        {
            if(DeepMatrix[i] < Dy)
            {
                Dy = DeepMatrix[i];
            }
            printf("%d,",DeepMatrix[i]);
            deepmatrix.DeepMatrix.push_back(DeepMatrix[i]);
        }
        printf("\n");
        printf("Dy = %d",Dy);
        

//////////////////////////////////////計算D && Dy/////////////////////////////////////////////

        
        printf("\n"),printf("V = ");    //計算V && XC
        for (int i = 0; i < DeepMatrixSize; i++)    
        {
            if (FocusMatrix[i] - DeepMatrix[i] > 0)
            {                                      
                FilterMatrix[i] = FocusMatrix[i] - DeepMatrix[i]; 
                printf("%d,",FilterMatrix[i]);
                Xc_cnt ++ ;
                Xc_n += i;
                Xc = Xc_n / Xc_cnt;
            }
            else 
            {
                FilterMatrix[i] = 0; 
                printf("%d,",FilterMatrix[i]);
            }
            
        }
        printf("\n");


//////////////////////////////////////計算V && XC/////////////////////////////////////////////            
        

        for(int i = 1; i <= 32; i++)
        {
            WR += (33-i) * FilterMatrix[i-1];
            WL += i * FilterMatrix[i-1];
        }
        printf("WR = %d",WR);
        printf("\n");
        printf("WL = %d",WL);
        printf("\n");
        if(WL < WR)
        {
            Xb = 0;
            printf("go go go RRRRRRRR");
        }
        else if(WR < WL)
        {
            Xb = 31;
            printf("go go go LLLLLLLL");
        }
        else if((WL > 0) && (WR > 0) && (WR == WL))
        {
            /*for(int i = 1;i <= 32;i++)
            {
                if(i <= 16)
                {
                    WL_sigma += DeepMatrix[i-1];
                }
                else
                {
                    WR_sigma += DeepMatrix[i-1];
                }
            }
            if(WL_sigma > WR_sigma)
            {
                WL += 10;
            }
            else
            {
                WR += 10;
            }*/
            printf("go go go go go go go go ");
            //printf("WL_sigma = %d,WR_sigma = %d",WL_sigma,WR_sigma);
            //printf("\n");
            //printf("WR = %d,WL = %d",WR,WL);
            //printf("\n");
        }
        Dx = Xc - Xb;                       //正負數可推算出障礙物位置
        printf("Dx = %f",Dx);


//////////////////////////////////////計算WR , WL && DX && Xb/////////////////////////////////////////////

        deepmatrix.Dx = Dx;
        deepmatrix.Dy = Dy;
        deepmatrix.WR = WR;
        deepmatrix.WL = WL;
        deepmatrix.Xc = Xc;
        deepmatrix.Xb = Xb;


        DeepMatrix_Publish.publish(deepmatrix);
        deepmatrix.DeepMatrix.clear();




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

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
            for(int i = 0;i < 32 ;i++)
            {
                if(DeepMatrix[i] < Dy)
                {
                    Dy = DeepMatrix[i];
                }
            }
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
            deepmatrix.DeepMatrix.push_back(DeepMatrix[i]);
            printf("%d,",DeepMatrix[i]);
        }
        printf("\n");
        printf("Dy = %d",Dy);

        DeepMatrix_Publish.publish(deepmatrix);
        deepmatrix.DeepMatrix.clear();
        
//////////////////////////////////////計算D && Dy/////////////////////////////////////////////

        Xc = 0;
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
        printf("Xc = %d",Xc);
        printf("\n");


//////////////////////////////////////計算V && XC/////////////////////////////////////////////            
        WR = 0;
        WL = 0;
        Dx = 0;
        for(int i = 1; i < 32; i++)
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
            Dx = abs(Xc - Xb);
            printf("go go go LLL");
        }
        else if(WR < WL)
        {
            Xb = 31;
            Dx = abs(Xc - Xb);
            printf("go go go RRR");
        }
        else
        {
            printf("go go go go go go go go ");
        }
        printf("\n");
        //Dx = abs(Xc - Xb);
        printf("Dx = %d",Dx);

//////////////////////////////////////計算WR , WL && DX && Xb/////////////////////////////////////////////

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

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
    INIT();

	if(strategy_info->getStrategyStart())
    {	
    //////////////////////////////////////顯示24X32中有沒有障礙物/////////////////////////////////////////////
        cv::Mat image = strategy_info->cvimg->image;

        for(int compress_height = 0 ; compress_height < IMAGEHEIGHT  ; compress_height++)   //顯示24X32中有沒有障礙物
        {
            for(int compress_width = 0 ; compress_width < IMAGEWIDTH ; compress_width++)
            {
                bValue = (image.data + ((compress_height*IMAGEWIDTH + compress_width) * 3 + 0));
                gValue = (image.data + ((compress_height*IMAGEWIDTH + compress_width) * 3 + 1));
                rValue = (image.data + ((compress_height*IMAGEWIDTH + compress_width) * 3 + 2));
                if((*rValue == 128 && *gValue == 0 && *bValue == 128) || (*rValue == 0 && *gValue == 128 && *bValue == 128))
                {
                    printf("1 ");
                }
                else
                {
                    printf("0 ");
                }
            }
            printf("\n");
        }
        

//////////////////////////////////////計算D && Dy/////////////////////////////////////////////


        printf("\n"),printf("D = ");                                //計算D && Dy                             
        for(int compress_width = 0 ; compress_width < IMAGEWIDTH  ; compress_width++)   
        {
            //DeepMatrix[compress_width] = 0;
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
        

//////////////////////////////////////計算V && XC/////////////////////////////////////////////

        
        for (int i = 0; i < 32; i++)    //計算V && XC
        {
            if (FocusMatrix[i] - DeepMatrix[i] > 0)
            {                                      
                FilterMatrix[i] = FocusMatrix[i] - DeepMatrix[i]; 
                Xc_cnt ++ ;
                Xc_n += i;
                Xc = Xc_n / Xc_cnt;
            }
            else 
            {
                FilterMatrix[i] = 0; 
            }
        }
        printf("\n");


//////////////////////////////////////計算WR , WL && DX && Xb/////////////////////////////////////////////     
        

        for(int i = 1; i <= 32; i++)
        {
            WR += (33-i) * FilterMatrix[i-1];
            WL += i * FilterMatrix[i-1];
        }

        if(WL < WR)
        {
            Xb = 0;
            printf("WR"),printf("\n");
        }
        else if(WR < WL)
        {
            Xb = 31;
            printf("WL"),printf("\n");
        }
        else if((WL > 0) && (WR > 0) && (WR == WL))
        {
            Xb = 0;
            /*for(int i = 0;i <= 32;i++)
            {
                if(i < 16)
                {
                    WL_sigma += DeepMatrix[i];
                }
                else
                {
                    WR_sigma += DeepMatrix[i];
                }
            }
            if(WL_sigma > WR_sigma)
            {
                WL += 10;
            }
            else if(WL_sigma < WR_sigma)
            {
                WR += 10;
            }
            printf("WL_sigma = %d,WR_sigma = %d",WL_sigma,WR_sigma);
            printf("\n");
            printf("WR = %d,WL = %d",WR,WL);
            printf("\n");*/
        }
        Dx = Xc - Xb;                       //正負數可推算出障礙物位置
        printf("Dx = %f",Dx);
        printf("\n"),printf("\n");


//////////////////////////////////////PUBLISH/////////////////////////////////////////////

        deepmatrix.Dx = Dx;
        deepmatrix.Dy = Dy;

        DeepMatrix_Publish.publish(deepmatrix);
        deepmatrix.DeepMatrix.clear();
    }
	else
	{
		
	} 

}
void OBSimage::INIT()
{
    Xc = 0;
    Xb = 0;
    WR = 0;
    WL = 0;
    WL_sigma = 0;
    WR_sigma = 0;
    Dx = 0;
    Xc_cnt = 0;
    Xc_n = 0;
    Dy = DeepMatrix[0];
}   

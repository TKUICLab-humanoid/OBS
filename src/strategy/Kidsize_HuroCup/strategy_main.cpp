#include "strategy/strategy_main.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OBSstrategy");
    ros::NodeHandle nh;
    KidsizeStrategy KidsizeStrategy(nh);

    ros::Rate loop_rate(30);

    KidsizeStrategy.initparameterpath();

    while (nh.ok())
    {
        KidsizeStrategy.strategymain();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

void KidsizeStrategy::strategymain()
{

    if (strategy_info->getStrategyStart()) //strategy start
    {
        for(int i = 0;i < 32;i++)
        {
            printf("%d, ",DeepMatrixValue[i]);
        }

        ROS_INFO("\n");

        ROS_INFO("weight_right = %d",weight_right);
        ROS_INFO("weight_left = %d",weight_left);
        ROS_INFO("nearest_distance_y = %d",nearest_distance_y);
        ROS_INFO("x_boundary = %d",x_boundary);
        ROS_INFO("obstacle_x_avg = %.3lf",obstacle_x_avg);
        ROS_INFO("x_avg_to_boundary = %.3lf",x_avg_to_boundary);
        
        ROS_INFO("\n\n\n\n\n\n\n");
    }
    else
    {

    }


}

void KidsizeStrategy::initparameterpath()
{
    while (parameter_path == "N")
    {
        parameter_path = tool->getPackagePath("strategy");
    }
    printf("parameter_path is %s\n", parameter_path.c_str());
}

void KidsizeStrategy::GetDeepMatrix(const strategy::DeepMatrix &msg)
{
    for (int i = 0; i < 32; i++)
    {
        DeepMatrixValue[i] = msg.DeepMatrix[i];
    }
    weight_right = msg.WR;
	weight_left = msg.WL;
	nearest_distance_y = msg.Dy;
	x_boundary = msg.Xb;

	obstacle_x_avg = msg.Xc;
	x_avg_to_boundary = msg.Dx;

}

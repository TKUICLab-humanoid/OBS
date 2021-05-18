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
        readwalkinggait();

        ROS_INFO("nearest_distance_y = %d",nearest_distance_y);
        ROS_INFO("x_avg_to_boundary = %.3lf",x_avg_to_boundary);
        ROS_INFO("[0] = %5d,[1] = %5d,[2] = %5d",dirdata[0],dirdata[1],dirdata[2]);
        ROS_INFO("[3] = %5d,[4] = %5d,[5] = %5d",dirdata[3],dirdata[4],dirdata[5]);
        ROS_INFO("[6] = %5d,[7] = %5d,[8] = %5d",dirdata[6],dirdata[7],dirdata[8]);
        ROS_INFO("\n");
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
	nearest_distance_y = msg.Dy;

	x_avg_to_boundary = msg.Dx;

}

void KidsizeStrategy::readwalkinggait() //步態參數之讀檔
{
    fstream fin;
    string sTmp;
    char line[100];
    char path[200];
    strcpy(path, parameter_path.c_str());
    strcat(path, "/WalkingGait.ini");
    fin.open(path, ios::in);
    char temp[100];
    try
    {
        fin.getline(temp, sizeof(temp));
        dirdata[0] = tool->readvalue(fin, "continuous_x_offset", 0);
        dirdata[1] = tool->readvalue(fin, "continuous_y_offset", 0);
        dirdata[2] = tool->readvalue(fin, "continuous_theta_offset", 0);
        dirdata[3] = tool->readvalue(fin, "continuous_x_offset_RIGHT", 0);
        dirdata[4] = tool->readvalue(fin, "continuous_y_offset_RIGHT", 0);
        dirdata[5] = tool->readvalue(fin, "continuous_theta_offset_RIGHT", 0);
        dirdata[6] = tool->readvalue(fin, "continuous_x_offset_LEFT", 0);
        dirdata[7] = tool->readvalue(fin, "continuous_y_offset_LEFT", 0);
        dirdata[8] = tool->readvalue(fin, "continuous_theta_offset_LEFT", 0);
        fin.close();
    }
    catch (exception e)
    {
    }
}
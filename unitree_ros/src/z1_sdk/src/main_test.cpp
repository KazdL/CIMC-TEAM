#include "unitree_arm_sdk/sensor/dataframe.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "main_test");
    ros::NodeHandle nh;
    Dataframe sensor_data;
    while(ros::ok())
    {
        ros::spinOnce();
    }
}
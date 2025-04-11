#include <unitree_ros/src/z1_sdk/include/unitree_arm_sdk/sensor/dataframe.h>

dataframe::dataframe(/* args */)
{
    wrench_sub = nh.subscribe("wrench", 10, &dataframe::wrenchCallback, this);
}

dataframe::~dataframe()
{
}

void dataframe::wrenchCallback(const geometry_msgs::Wrench::ConstPtr& msg) {
        force = msg->force;
        torque = msg->torque;
        ROS_INFO_STREAM("Force: [" << force.x << ", " << force.y << ", " << force.z << "]");
        ROS_INFO_STREAM("Torque: [" << torque.x << ", " << torque.y << ", " << torque.z << "]");
    }

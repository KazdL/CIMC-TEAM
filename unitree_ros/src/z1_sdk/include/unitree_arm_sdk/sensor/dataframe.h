#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include "unitree_arm_sdk/control/unitreeArm.h"

class Dataframe
{
private:
    Vec6 ee_force;
    ros::NodeHandle nh;                   // ROS节点句柄
    ros::Subscriber wrench_sub;           // 订阅者
    geometry_msgs::Vector3 force, torque;  // 用于存储力和力矩
public:
    Dataframe(/* args */);
    ~Dataframe();
    void wrenchCallback(const geometry_msgs::Wrench::ConstPtr& msg);
};

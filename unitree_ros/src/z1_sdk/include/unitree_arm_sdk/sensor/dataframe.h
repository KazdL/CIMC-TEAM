#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>

class dataframe
{
private:
    float fx,fy,fz,tx,ty,tz;
    ros::NodeHandle nh;                  // ROS节点句柄
    ros::Subscriber wrench_sub;           // 订阅者
    geometry_msgs::Vector3 force, torque;  // 用于存储力和力矩
public:
    dataframe(/* args */);
    ~dataframe();
    void wrenchCallback(const geometry_msgs::Wrench::ConstPtr& msg);
};

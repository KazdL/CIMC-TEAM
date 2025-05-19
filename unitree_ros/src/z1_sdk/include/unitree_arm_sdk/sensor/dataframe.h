#include <iostream>
#include "unitree_arm_sdk/control/Velocity_planning.h"
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <Eigen/Dense>

class Dataframe
{
private:
    ros::NodeHandle nh;                   // ROS节点句柄
    ros::Subscriber wrench_sub;           // 订阅者
    geometry_msgs::Vector3 force, torque; // 用于存储力和力矩
    Eigen::Matrix3d ee_rotation;          // 末端执行器旋转矩阵

public:
    Vec6 ee_force, ee_pos, ee_vel;
    Vec6 cur_joint_pos, cur_joint_vel;

    Dataframe(/* args */);
    ~Dataframe();
    void wrenchCallback(const geometry_msgs::Wrench::ConstPtr& msg);
    void update(const Vel_Planning &planner);
};


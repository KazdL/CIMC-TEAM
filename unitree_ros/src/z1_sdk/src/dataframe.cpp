#include "unitree_arm_sdk/sensor/dataframe.h"

Dataframe::Dataframe(/* args */)
{
    ROS_INFO("dataframe created");
    wrench_sub = nh.subscribe("wrench", 10, &Dataframe::wrenchCallback, this);
}

Dataframe::~Dataframe()
{
}

void Dataframe::wrenchCallback(const geometry_msgs::Wrench::ConstPtr& msg) {
    ROS_INFO("callback success");
    force = msg->force;
    torque = msg->torque;
    ee_force[0] = force.x;
    ee_force[1] = force.y;
    ee_force[2] = force.z;
    ee_force[3] = torque.x;
    ee_force[4] = torque.y;
    ee_force[5] = torque.z;
    ROS_INFO_STREAM("Force: [" << force.x << ", " << force.y << ", " << force.z << "]");
    ROS_INFO_STREAM("Torque: [" << torque.x << ", " << torque.y << ", " << torque.z << "]");
}

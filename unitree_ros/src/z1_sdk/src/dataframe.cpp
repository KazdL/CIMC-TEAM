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

void Dataframe::update(const Vel_Planning& planner)
{
    cur_joint_pos = planner.lowstate->getQ();
    cur_joint_vel = planner.lowstate->getQd();
    HomoMat T = planner._ctrlComp->armModel->forwardKinematics(cur_joint_pos);
    ee_pos = homoToPosture(T);
    Mat6 J = planner._ctrlComp->armModel->CalcJacobian(cur_joint_pos);
    ee_vel = J * cur_joint_vel;
}
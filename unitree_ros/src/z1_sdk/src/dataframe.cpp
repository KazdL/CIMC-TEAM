#include "unitree_arm_sdk/sensor/dataframe.h"

Dataframe::Dataframe(/* args */)
{
    ROS_INFO("dataframe created");
    wrench_sub = nh.subscribe("wrench", 10, &Dataframe::wrenchCallback, this);
    ee_rotation.setIdentity(); // Initialize with identity matrix
}

Dataframe::~Dataframe()
{
}

void Dataframe::wrenchCallback(const geometry_msgs::Wrench::ConstPtr& msg)
{
    ROS_INFO("callback success");
    
    // 1. Get raw force/torque in end-effector frame
    Eigen::Vector3d local_force(msg->force.x, msg->force.y, msg->force.z);
    Eigen::Vector3d local_torque(msg->torque.x, msg->torque.y, msg->torque.z);
    
    // 2. Transform to world frame
    Eigen::Vector3d world_force = ee_rotation * local_force;
    Eigen::Vector3d world_torque = ee_rotation * local_torque;
    
    // 3. Store in member variables
    force.x = world_force.x();
    force.y = world_force.y();
    force.z = world_force.z();
    
    torque.x = world_torque.x();
    torque.y = world_torque.y();
    torque.z = world_torque.z();
    
    // 4. Store in ee_force array
    ee_force << world_torque.x(), world_torque.y(), world_torque.z(), world_force.x(), world_force.y(), world_force.z();

    ROS_INFO_STREAM("World Frame Force: [" << force.x << ", " << force.y << ", " << force.z << "]");
    ROS_INFO_STREAM("World Frame Torque: [" << torque.x << ", " << torque.y << ", " << torque.z << "]");
}

void Dataframe::update(const Vel_Planning &planner)
{
 
    cur_joint_pos = planner.lowstate->getQ();
    cur_joint_vel = planner.lowstate->getQd();
    HomoMat T = planner._ctrlComp->armModel->forwardKinematics(cur_joint_pos);
    ee_pos = homoToPosture(T);  // Assuming this extracts position
    ee_rotation = T.block<3,3>(0,0); // Extract rotation matrix
    Mat6 J = planner._ctrlComp->armModel->CalcJacobian(cur_joint_pos);
    ee_vel = J * cur_joint_vel;
}
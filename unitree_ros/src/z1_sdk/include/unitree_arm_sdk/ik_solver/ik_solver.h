#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <iostream>

class ik_solver {
public:
    // 构造函数
    ik_solver(const std::string &urdf_filename);

    // 求解逆运动学
    bool solve(const pinocchio::SE3 &target_pose, Eigen::VectorXd &result, Eigen::VectorXd init_pose, int joint_id = -1);

    // 获取关节数量
    int getNumberOfJoints() const;

    // 更新末端姿态
    void updateEndEffectorPose(int joint_id);

    // 获取末端姿态
    pinocchio::SE3 getEndEffectorPose() const;

private:
    pinocchio::Model model;           // 模型
    pinocchio::Data data;             // 数据
    bool model_loaded;                // 是否加载模型
    std::string urdf_filename;        // URDF文件名
    pinocchio::SE3 end_effector_pose; // 末端姿态
};

#endif // IK_SOLVER_H

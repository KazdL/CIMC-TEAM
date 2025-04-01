#ifndef IK_SOLVER_H
#define IK_SOLVER_H

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include <iostream>

class ik_solver {
public:
    ik_solver(const std::string &urdf_filename);

    bool solve(const pinocchio::SE3 &target_pose, Eigen::VectorXd &result, const Eigen::VectorXd &init_pose = Eigen::VectorXd(), int joint_id = -1);

    int getNumberOfJoints() const;

private:
    pinocchio::Model model;
    pinocchio::Data data;
    bool model_loaded;
    std::string urdf_filename;
};

#endif // IK_SOLVER_H

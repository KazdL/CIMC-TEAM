#include "unitree_arm_sdk/ik_solver/ik_solver.h"

ik_solver::ik_solver(const std::string &urdf_filename)
    : model_loaded(false), urdf_filename(urdf_filename) {
    try {
        pinocchio::urdf::buildModel(urdf_filename, model);
        data = pinocchio::Data(model);
        model_loaded = true;
        std::cout << "Successfully loaded model from URDF." << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "Error loading URDF file: " << e.what() << std::endl;
    }
}

bool ik_solver::solve(const pinocchio::SE3 &target_pose, Eigen::VectorXd &result, const Eigen::VectorXd &init_pose, int joint_id) {
    if (!model_loaded) {
        std::cerr << "Model not loaded. Cannot solve IK." << std::endl;
        return false;
    }

    if (joint_id == -1) {
        joint_id = model.njoints - 1;
    }

    Eigen::VectorXd q = init_pose.size() == 0 ? pinocchio::neutral(model) : init_pose;
    const double eps = 1e-4;
    const int IT_MAX = 1000;
    const double DT = 1e-1;
    const double damp = 1e-12;

    pinocchio::Data::Matrix6x joint_jacobian(6, model.nv);
    joint_jacobian.setZero();

    bool success = false;
    Eigen::Vector3d err;
    Eigen::VectorXd v(model.nv);

    for (int i = 0; i < IT_MAX; ++i) {
        pinocchio::forwardKinematics(model, data, q);
        const pinocchio::SE3 iMd = data.oMi[joint_id].actInv(target_pose);
        err = iMd.translation();

        if (err.norm() < eps) {
            success = true;
            break;
        }

        pinocchio::computeJointJacobian(model, data, q, joint_id, joint_jacobian);
        const auto J = -joint_jacobian.topRows<3>();
        const Eigen::Matrix3d JJt = J * J.transpose() + damp * Eigen::Matrix3d::Identity();
        v.noalias() = -J.transpose() * JJt.ldlt().solve(err);
        q = pinocchio::integrate(model, q, v * DT);

        if (i % 10 == 0) {
            std::cout << i << ": error = " << err.transpose() << std::endl;
        }
    }

    if (success) {
        std::cout << "Convergence achieved!" << std::endl;
    } else {
        std::cerr << "Warning: the iterative algorithm did not converge." << std::endl;
    }

    result = q;
    std::cout << "Final configuration: " << q.transpose() << std::endl;
    std::cout << "Final error: " << err.transpose() << std::endl;

    return success;
}

int ik_solver::getNumberOfJoints() const {
    if (!model_loaded) {
        std::cerr << "Model not loaded. Cannot get number of joints." << std::endl;
        return -1;
    }
    return model.njoints;
}

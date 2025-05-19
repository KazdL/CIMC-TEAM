#ifndef ADMITTANCE_CONTROLLER_H
#define ADMITTANCE_CONTROLLER_H

#include "unitree_arm_sdk/control/unitreeArm.h"

class AdmittanceController {
public:
    AdmittanceController(
        const Vec6& d_mass = Vec6::Constant(1.0),
        const Vec6& d_stiffness = (Vec6() << 400.0, 400.0, 400.0, 400.0, 400.0, 400.0).finished(),
        double d_damping_ratio = 1.0,
        const Vec6& d_stiffness_force = Vec6::Zero(),
        double timestep = 0.005,
        const Vec6& fd = (Vec6() << 0, 0, 0, 0, 0, 0).finished()
    );

    Vec6 update_pos(Vec6& pos, Vec6& vel, const Vec6& xd, const Vec6& d_xd, const Vec6& dd_xd, const Vec6& fd);

private:
    Vec6 Md, Kd, Dd, Kf;
    Vec6 force_err, pos_err, vel_err;
    Vec6 ee_pos, ee_vel, ee_force;
    Vec6 cart_acc_append, vel_append, pos_append;
    double damping_ratio;
    double timestep;
    Vec6 fd;
};

#endif // ADMITTANCE_CONTROLLER_H

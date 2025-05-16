#include "unitree_arm_sdk/control/AdmittanceController.h"

AdmittanceController::AdmittanceController(
    const Vec6& d_mass,
    const Vec6& d_stiffness,
    double d_damping_ratio,
    const Vec6& d_stiffness_force,
    double timestep
)
    : Md(d_mass), Kd(d_stiffness), damping_ratio(d_damping_ratio), Kf(d_stiffness_force), timestep(timestep) {
    Dd = damping_ratio * 2 * Md.cwiseSqrt().cwiseProduct(Kd.cwiseSqrt());

    force_err = Vec6::Zero();
    pos_err = Vec6::Zero();
    vel_err = Vec6::Zero();
    
    ee_pos = Vec6::Zero();
    ee_vel = Vec6::Zero();
    ee_force = Vec6::Zero();
    
    cart_acc_append = Vec6::Zero();
    vel_append = Vec6::Zero();
    pos_append = Vec6::Zero();
}

Vec6 AdmittanceController::update_pos(const Vec6& pos, const Vec6& vel, const Vec6& xd, const Vec6& d_xd, const Vec6& dd_xd, const Vec6& fd) {
    // Compute position and velocity errors
    pos_err = pos - xd;
    vel_err = vel - d_xd;
    pos_err -= (-d_xd) * timestep;
    vel_err -= (-dd_xd) * timestep;

    force_err = ee_force - fd;

    // Compute cartesian acceleration append element-wise
    for (int i = 0; i < 6; ++i) {
        cart_acc_append[i] = (1.0 / Md[i]) * (-Dd[i] * vel_err[i] - Kd[i] * pos_err[i] + Kf[i] * force_err[i]);
    }

    // Update velocity and position append
    vel_append = vel_err + cart_acc_append * timestep;
    pos_append = pos_err + vel_append * timestep;

    Vec6 result = xd + pos_append;

    return result;
}

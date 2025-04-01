#ifndef _VELOCITY_PLANNING_H_
#define _VELOCITY_PLANNING_H_

#include "unitree_arm_sdk/control/unitreeArm.h"
#include "unitree_arm_sdk/ik_solver/ik_solver.h"

using namespace UNITREE_ARM;

class Vel_Planning : public unitreeArm
{
public:
    Vel_Planning() : unitreeArm(true), solver("src/z1_sdk/include/unitree_arm_sdk/ik_solver/z1.urdf") {};
    ~Vel_Planning() {};
    void trapezium(Vec6 targetpos, double duration);

    void trape_move_cart(Vec6 targetpos);
    void trape_move_joint(Vec6 q_end);
    void test_MoveL_sin();
    void test_linear();
    void move_in_Cartesian(Vec6 targetpos, Vec6 targetvel);
    double max_vel = 0.3;
    double max_acc = 1.0;
    const std::string urdf_path = "unitree_arm_sdk/ik_solver/z1.urdf";
    ik_solver solver;


    std::list<Vec6> cmd_Pos_list, cmd_Vel_list;
};

#endif
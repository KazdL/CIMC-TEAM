#include "unitree_arm_sdk/control/Velocity_planning.h"
#include "unitree_arm_sdk/sensor/dataframe.h"
#include "ros/ros.h"
#include "unitree_arm_sdk/control/AdmittanceController.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;

    // objects definition
    Vel_Planning planner;
    Dataframe sensor_data;
    UNITREE_ARM::Timer timer(planner._ctrlComp->dt);

    Vec6 d_mass = Vec6::Constant(1.0); // 默认质量
    Vec6 d_stiffness;
    d_stiffness << 400.0, 400.0, 400.0, 400.0, 400.0, 400.0; // 默认刚度
    double d_damping_ratio = 1.0; // 默认阻尼比
    Vec6 d_stiffness_force = Vec6::Zero(); // 默认力刚度
    double timestep = 0.005; // 时间步长
    AdmittanceController controller(d_mass, d_stiffness, d_damping_ratio, d_stiffness_force, timestep);

    // variables definition 
    Vec6 targetpos;
    Vec6 cmdPos, cmdVel;
    Vec6 xd, d_xd, dd_xd;
    bool motion_ready = true;
    std::string status = "init";

    // robot action init
    planner.sendRecvThread->start();
    planner.backToStart();
    planner.startTrack(UNITREE_ARM::ArmFSMState::JOINTCTRL);

    std::list<Vec6> xd_list, dxd_list, ddxd_list;
    

    while(ros::ok())
    {
        if(motion_ready)
        {
            if(status == "init")
            {
                targetpos << 0.0, 1.5, -1.0, -0.54, 0.0, 0.0;
                HomoMat target_T = planner._ctrlComp->armModel->forwardKinematics(targetpos);
                Vec6 target = homoToPosture(target_T);
                // planner.trapezium(targetpos, 4000);
                // planner.trape_move_joint(targetpos);
                planner.trape_move_cart(target);
                // planner.test_linear();
                status = "terminate";
            }
            else if (status == "terminate")
            {
                break;
            }
            motion_ready = false;
        }
        else{
            if (planner.cmd_Pos_list.size() != 0)
            {
                cmdPos = planner.cmd_Pos_list.front();
                cmdVel = planner.cmd_Vel_list.front();
                planner.cmd_Pos_list.pop_front();
                planner.cmd_Vel_list.pop_front();
            }
            else
            {
                cmdVel.Zero();
                xd = xd_list.front();
                d_xd = dxd_list.front();
                dd_xd = ddxd_list.front();
                xd_list.pop();
                dxd_list.pop();
                ddxd_list.pop();
                target = controller.update_pos(cmdPos, cmdVel, xd, d_xd, dd_xd, sensor_data.ee_force);
                planner.trape_move_cart(target);
                motion_ready = true;
            }
        }
        planner.setArmCmd(cmdPos, cmdVel);
        timer.sleep();
    }
    planner.backToStart();
    planner.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    planner.sendRecvThread->shutdown();
    return 0;
}
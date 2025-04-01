#include "unitree_arm_sdk/control/Velocity_planning.h"
#include "ros/ros.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;

    Vel_Planning planner;
    planner.sendRecvThread->start();
    planner.backToStart();
    planner.startTrack(UNITREE_ARM::ArmFSMState::JOINTCTRL);
    Vec6 targetpos;
    UNITREE_ARM::Timer timer(planner._ctrlComp->dt);
    Vec6 cmdPos, cmdVel;
    bool motion_ready = true;
    std::string status = "init";

    
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
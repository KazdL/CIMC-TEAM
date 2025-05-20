#include "unitree_arm_sdk/control/Velocity_planning.h"
#include "unitree_arm_sdk/sensor/dataframe.h"
#include "ros/ros.h"
#include "unitree_arm_sdk/control/Admittance.h"


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "main_node");
    ros::NodeHandle nh;

    // objects definition
    Vel_Planning planner;
    Dataframe dataframe;
    UNITREE_ARM::Timer timer(planner._ctrlComp->dt);

    Vec6 d_mass = Vec6::Constant(1.0); // 默认质量
    Vec6 d_stiffness;
    d_stiffness << 400.0, 400.0, 400.0, 400.0, 400.0, 400.0; // 默认刚度
    double d_damping_ratio = 1.0; // 默认阻尼比
    Vec6 d_stiffness_force = Vec6::Constant(50); // 默认力刚度
    double timestep = planner._ctrlComp->dt; // 时间步长
    AdmittanceController controller(d_mass, d_stiffness, d_damping_ratio, d_stiffness_force, timestep);

    // variables definition 
    Vec6 targetpos;
    Vec6 target;
    Vec6 cmdPos, cmdVel;
    Vec6 xd, d_xd, dd_xd;
    std::list<Vec6> xd_list, dxd_list, ddxd_list;
    bool motion_ready = true;
    std::string status = "init";
    std::string last_status = "init";
    std::string stat_chg_request = "init";
    // robot action init
    planner.sendRecvThread->start();
    planner.backToStart();
    planner.startTrack(UNITREE_ARM::ArmFSMState::JOINTCTRL);
    

    
    

    while(ros::ok())
    {
        dataframe.update(planner);
        
        // std::cout << "cur_joint_Vel:" << dataframe.ee_vel.transpose() << std::endl;
        if (motion_ready)
        {
            if(status == "init")
            {
                targetpos << 0.0, 1.5, -1.0, -0.54, 0.0, 0.0;
                HomoMat target_T = planner._ctrlComp->armModel->forwardKinematics(targetpos);
                target = homoToPosture(target_T);
                // planner.trapezium(targetpos, 4000);
                // planner.trape_move_joint(targetpos);
                planner.trape_move_cart(target);
                // planner.test_linear();
                
                stat_chg_request = "admit";
            }
            else if (status == "admit")
            {
                // xd = xd_list.front();
                // d_xd = dxd_list.front();
                // dd_xd = ddxd_list.front();
                if(last_status != "admit")
                {
                    planner.startTrack(UNITREE_ARM::ArmFSMState::CARTESIAN);
                    last_status = "admit";
                }
                    
                xd = target;
                std::cout << xd.transpose() << std::endl;
                d_xd = Vec6::Zero();
                dd_xd = Vec6::Zero();
                
                // xd_list.pop();
                // dxd_list.pop();
                // ddxd_list.pop();
                Vec6 cmd_cart_Pos = dataframe.ee_pos;
                Vec6 cmd_cart_Vel = dataframe.ee_vel;
                Vec6 admit_pos = controller.update_pos(cmd_cart_Pos, cmd_cart_Vel, xd, d_xd, dd_xd, dataframe.ee_force);
                std::cout << "cmdPos:" << cmd_cart_Pos.transpose() << std::endl;
                std::cout << "cmdVel:" << cmd_cart_Vel.transpose() << std::endl;
                // planner.move_in_Cartesian(cmd_cart_Pos, cmd_cart_Vel, dataframe.cur_joint_pos, dataframe.cur_joint_vel);
                Vec7 direction = Vec7::Zero();
                for (int i = 0; i < 6; i++)
                {
                    /* code */
                    if(i<3)
                    {
                        direction[i] = saturation(cmd_cart_Vel[i] / 0.6, -1.0, 1.0);
                    }
                    else{
                        direction[i] = saturation(cmd_cart_Vel[i] / 0.3, -1.0, 1.0);
                    }
                }
                std::cout << "cmd_direction" << direction.transpose() << std::endl;
                planner.cartesianCtrlCmd(direction, 0.6, 0.3);
                // ros::spinOnce();
                // timer.sleep();
                // continue;
                // planner.MoveL(cmd_cart_Pos, 0.3);
            }
            else if (status == "terminate")
            {
                break;
            }
            motion_ready = false;
        }
        if(!motion_ready){
            if (planner.cmd_Pos_list.size() != 0)
            {
                Vec6 cur_vel = planner.lowstate->getQd();
                cmdPos = planner.cmd_Pos_list.front();
                cmdVel = planner.cmd_Vel_list.front();
                planner.cmd_Pos_list.pop_front();
                planner.cmd_Vel_list.pop_front();
            }
            else
            {
                cmdVel.Zero();
                motion_ready = true;
                if(stat_chg_request != status)
                    status = stat_chg_request;
            }
        }
        if(status != "admit")
            planner.setArmCmd(cmdPos, cmdVel);
        ros::spinOnce();
        timer.sleep();
    }
    planner.backToStart();
    planner.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    planner.sendRecvThread->shutdown();
    return 0;
}
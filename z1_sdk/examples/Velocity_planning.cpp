#include "unitree_arm_sdk/control/unitreeArm.h"
#include "unitree_arm_sdk/ik_solver/ik_solver.h"

using namespace UNITREE_ARM;

class Vel_Planning : public unitreeArm{
public:
    Vel_Planning() : unitreeArm(true){};
    ~Vel_Planning() {};
    std::vector<Vec6> trapezium(Vec6 targetpos, double duration);

    void trape_move_cart(Vec6 targetpos);
    void trape_move_joint(Vec6 q_end);
    void test_MoveL_sin();
    void test_linear();
    void move_in_Cartesian(Vec6 targetpos, Vec6 targetvel);
    double max_vel = 0.3;
    double max_acc = 1.0;
    const std::string urdf_path = "unitree_arm_sdk/ik_solver/z1.urdf";
    // ik_solver solver;
};

std::vector<Vec6> Vel_Planning::trapezium(Vec6 targetpos, double duration)
{
    startTrack(ArmFSMState::JOINTCTRL);

    double dura = duration * _ctrlComp->dt;

    Vec6 cur_pos = lowstate->getQ();
    for (int i = 0; i < 6;i++)
    {
        if (abs(targetpos[i]-cur_pos[i])> dura * max_vel)
        {
            std::cout << "duration time too low!!" << std::endl;
            return std::vector<Vec6>(0);
        }
    }

    Vec6 cmdVel, cmdPos = cur_pos;
    std::vector<Vec6> cmd_Pos_list;
    std::vector<std::vector<double>> diff_time;
    Timer timer(_ctrlComp->dt);

    for (int i = 0; i < 6; i++)
    {
        double delta_pos = abs(targetpos[i] - cur_pos[i]);
        double t1, t2;
        double upper_vel;
        upper_vel = 4.0 / 3.0 * delta_pos / dura;
        if(upper_vel>max_vel)
            upper_vel = max_vel;
        t1 = dura / 2 - (delta_pos * 2 / upper_vel - dura) / 2;
        t2 = dura / 2 + (delta_pos * 2 / upper_vel - dura) / 2;
        diff_time.push_back({t1, t2, upper_vel});
        std::cout << t1 << ":" << t2 << ":" << upper_vel << std::endl;
    }
    for (int j = 0; j < duration; j++)
    {
        for (int i = 0; i < 6; i++)
        {
            switch ((j * _ctrlComp->dt > diff_time[i][0]) + (j * _ctrlComp->dt > diff_time[i][1]))
            {
            case 0:
                /* code */
                cmdVel(i, 0) = (targetpos[i] - cur_pos[i]) / abs(targetpos[i] - cur_pos[i]) * j * _ctrlComp->dt / diff_time[i][0] * diff_time[i][2];
                cmdPos(i, 0) += cmdVel(i, 0) * _ctrlComp->dt;
                break;

            case 1:
                cmdVel(i, 0) = (targetpos[i] - cur_pos[i]) / abs(targetpos[i] - cur_pos[i]) * diff_time[i][2];
                cmdPos(i, 0) += cmdVel(i, 0) * _ctrlComp->dt;
                break;

            case 2:
                cmdVel(i, 0) = (targetpos[i] - cur_pos[i]) / abs(targetpos[i] - cur_pos[i]) * (1-(j * _ctrlComp->dt - diff_time[i][1]) / (dura - diff_time[i][1])) * diff_time[i][2];
                cmdPos(i, 0) += cmdVel(i, 0) * _ctrlComp->dt;
                break;

            default : 
                break;
            }
        }
        cmd_Pos_list.push_back(cmdPos);
        setArmCmd(cmdPos, cmdVel);
        timer.sleep();
    }
    return cmd_Pos_list;
}


void Vel_Planning::trape_move_joint(Vec6 q_end)
{
    Vec6 q_start = lowstate->getQ();
    double duration = 2000;
}

void Vel_Planning::trape_move_cart(Vec6 targetpos)
{
    Vec6 cur_joint_pos = lowstate->getQ();
    HomoMat target_T = postureToHomo(targetpos);
    Vec6 tar_joint_pos;
    bool invisok = _ctrlComp->armModel->inverseKinematics(target_T, cur_joint_pos, tar_joint_pos, true);
    std::cout << tar_joint_pos<<std::endl;
    trapezium(tar_joint_pos, 4000);
}

void Vel_Planning::test_MoveL_sin()
{
    Vec6 cur_joint_pos = lowstate->getQ();
    HomoMat cur_T = _ctrlComp->armModel->forwardKinematics(cur_joint_pos);
    Vec6 cur_pos = homoToPosture(cur_T);
    Timer timer(_ctrlComp->dt);
    Vec6 last_pos;
    Vec6 tar_vel = lowstate->getQd();
    for (int i = 0; i < 1000; i++)
    {
        last_pos = cur_pos;
        cur_pos(4, 0) += 10* 0.3 * _ctrlComp->dt;
        cur_pos(5, 0) = last_pos[4, 0] + 10 * sin(_ctrlComp->dt*i);
        tar_vel(4, 0) = 10*0.3;
        tar_vel(5, 0) = 10 * cos(_ctrlComp->dt*i);
        move_in_Cartesian(cur_pos, tar_vel);
        timer.sleep();
    }
}

void Vel_Planning::test_linear()
{
    Vec6 cur_joint_pos = lowstate->getQ();
    HomoMat cur_T = _ctrlComp->armModel->forwardKinematics(cur_joint_pos);
    Vec6 cur_pos = homoToPosture(cur_T);
    std::cout << cur_pos;
    Vec6 last_pos;
    cur_pos[5] += 0.3;
    trape_move_cart(cur_pos);
}

void Vel_Planning::move_in_Cartesian(Vec6 targetpos, Vec6 targetvel)
{
    Vec6 last_joint_pos = lowstate->getQ();
    Vec6 test_pos;
    test_pos << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    Vec6 last_joint_vel = lowstate->getQd();
    HomoMat target_T = postureToHomo(targetpos);
    Vec6 tar_joint_pos, tar_joint_vel;
    bool invisok = _ctrlComp->armModel->inverseKinematics(target_T, last_joint_pos, tar_joint_pos);
    _ctrlComp->armModel->solveQP(targetvel, last_joint_vel, tar_joint_vel, _ctrlComp->dt);
    if (invisok)std::cout << "target postion:" << tar_joint_pos << std::endl;
    else
        std::cout << "No solution found!!" << std::endl;
    setArmCmd(tar_joint_pos, tar_joint_vel);
}

int main()
{

    Vel_Planning planner;
    planner.sendRecvThread->start();
    planner.backToStart();
    Vec6 targetpos;
    targetpos << 0.0, 1.5, -1.0, -0.54, 0.0, 0.0;
    planner.trapezium(targetpos, 4000);
    // planner.trape_move_joint(targetpos);

    // planner.test_linear();

    planner.backToStart();
    planner.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    planner.sendRecvThread->shutdown();
    return 0;
}
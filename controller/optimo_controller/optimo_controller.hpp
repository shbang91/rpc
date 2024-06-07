#pragma once
#include "configuration.hpp"
#include "util/util.hpp"

#if B_USE_MATLOGGER
#include <matlogger2/matlogger2.h>
#endif

class PinocchioRobotSystem;
class OptimoTCIContainer;
class OptimoStateProvider;
class IHWBC;
class JointIntegrator;

class OptimoController {
public:
    OptimoController(OptimoTCIContainer *tci_container,
                     PinocchioRobotSystem *robot);
    virtual ~OptimoController();

    void GetCommand(void *command);

private:
    PinocchioRobotSystem *robot_;
    OptimoTCIContainer *tci_container_;
    OptimoStateProvider *sp_;

    IHWBC *ihwbc_;
    JointIntegrator *joint_integrator_;

    Eigen::VectorXd joint_pos_cmd_;
    Eigen::VectorXd joint_vel_cmd_;
    Eigen::VectorXd joint_trq_cmd_;
    Eigen::VectorXd joint_trq_cmd_prev_;
    Eigen::VectorXd wbc_qddot_cmd_;

    bool b_sim_;

    bool b_first_visit_wbc_ctrl_;
    bool b_first_visit_pos_ctrl_;
    bool b_smoothing_command_;
    double smoothing_command_duration_;
    double smoothing_command_start_time_;

    Eigen::VectorXd init_joint_pos_;
    Eigen::MatrixXd sa_; // selection matrix

    void _SaveData();

#if B_USE_MATLOGGER
  XBot::MatLogger2::Ptr logger_;
#endif
};
#pragma once
#include "configuration.hpp"
#include "util/util.hpp"

#if B_USE_MATLOGGER
#include <matlogger2/matlogger2.h>
#endif

class PinocchioRobotSystem;
class Med7TCIContainer;
class Med7StateProvider;
class IHWBC;
class JointIntegrator;

class Med7Controller {
public:
    Med7Controller(Med7TCIContainer *tci_container,
                     PinocchioRobotSystem *robot);
    virtual ~Med7Controller();

    void GetCommand(void *command);

private:
    PinocchioRobotSystem *robot_;
    Med7TCIContainer *tci_container_;
    Med7StateProvider *sp_;

    IHWBC *ihwbc_;
    JointIntegrator *joint_integrator_;

    Eigen::VectorXd joint_pos_cmd_;
    Eigen::VectorXd joint_vel_cmd_;
    Eigen::VectorXd joint_trq_cmd_;
    Eigen::VectorXd joint_trq_cmd_prev_;
    Eigen::VectorXd wbc_qddot_cmd_;

    bool b_sim_;
    bool b_int_constraint_first_visit_;

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

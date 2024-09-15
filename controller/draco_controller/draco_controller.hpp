#pragma once
#include "configuration.hpp"
#include "util/util.hpp"

#if B_USE_MATLOGGER
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#endif

class PinocchioRobotSystem;
class DracoTCIContainer;
class DracoStateProvider;
class JointIntegrator;
class WBC;

class DracoController {
public:
  DracoController(DracoTCIContainer *tci_container, PinocchioRobotSystem *robot,
                  const YAML::Node &cfg);
  virtual ~DracoController();

  void GetCommand(void *command);

private:
  PinocchioRobotSystem *robot_;
  DracoTCIContainer *tci_container_;
  DracoStateProvider *sp_;

  // TODO: combine these two
  // IHWBC
  WBC *ihwbc_;
  JointIntegrator *joint_integrator_;
  // WBIC
  WBC *wbic_;

  Eigen::VectorXd joint_pos_cmd_;
  Eigen::VectorXd joint_vel_cmd_;
  Eigen::VectorXd joint_trq_cmd_;
  Eigen::VectorXd joint_trq_cmd_prev_;
  Eigen::VectorXd wbc_qddot_cmd_;

  bool b_use_modified_swing_foot_jac_;
  bool b_use_modified_hand_jac_;

  bool b_first_visit_pos_ctrl_;
  Eigen::VectorXd init_joint_pos_;
  bool b_first_visit_wbc_ctrl_;
  bool b_int_constraint_first_visit_;
  bool b_smoothing_command_;
  double smoothing_command_duration_;
  double smoothing_command_start_time_;
  bool b_use_filtered_torque_;
  double alpha_cmd_;

  void _SaveData();

#if B_USE_MATLOGGER
  XBot::MatLogger2::Ptr logger_;
  XBot::MatAppender::Ptr appender_;
#endif
};

#pragma once
#include "util/util.hpp"

class PinocchioRobotSystem;
class DracoTCIContainer;
class DracoStateProvider;
class IHWBC;

class DracoController {
public:
  DracoController(DracoTCIContainer *tci_container,
                  PinocchioRobotSystem *robot);
  virtual ~DracoController();

  void GetCommand(void *command);

private:
  PinocchioRobotSystem *robot_;
  DracoTCIContainer *tci_container_;
  DracoStateProvider *sp_;

  IHWBC *ihwbc_;

  Eigen::VectorXd joint_pos_cmd_;
  Eigen::VectorXd joint_vel_cmd_;
  Eigen::VectorXd joint_trq_cmd_;

  bool b_int_constrinat_first_visit_;

  bool b_first_visit_;
  bool b_smoothing_command_;
  double smoothing_command_duration_;
  double smoothing_command_start_time_;
  Eigen::VectorXd init_joint_pos_;

  int data_save_freq_;

  void _SaveData();
};

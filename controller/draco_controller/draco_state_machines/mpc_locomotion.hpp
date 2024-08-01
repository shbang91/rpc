#pragma once
#include "controller/state_machine.hpp"

class PinocchioRobotSystem;
class DracoControlArchitecture;
class DracoStateProvider;
class GaitCommand;

class MPCLocomotion : public StateMachine {
public:
  MPCLocomotion(const StateId state_id, PinocchioRobotSystem *robot,
                DracoControlArchitecture *ctrl_arch);
  virtual ~MPCLocomotion() = default;

  void FirstVisit() override;
  void OneStep() override;
  bool EndOfState() override;
  void LastVisit() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

  // interrupt commands
  bool b_increase_x_vel_ = false;
  bool b_increase_y_vel_ = false;
  bool b_increase_yaw_vel_ = false;
  bool b_decrease_yaw_vel_ = false;

private:
  DracoControlArchitecture *ctrl_arch_;
  DracoStateProvider *sp_;

  // gait
  std::shared_ptr<GaitCommand> gait_command_;

  // contact state
  Eigen::Vector2d prev_contact_states_;

  // WBC QP params yaml
  Eigen::VectorXd W_force_rate_of_change_left_foot_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd W_force_rate_of_change_right_foot_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd W_delta_rf_lfoot_ = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd W_delta_rf_rfoot_ = Eigen::VectorXd::Zero(6);
};

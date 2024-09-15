#pragma once

#include "controller/state_machine.hpp"

class PinocchioRobotSystem;
class DracoControlArchitecture;
class DracoStateProvider;
class TeleopHandler;
class DracoRSCommands;

class TeleopManipulation : public StateMachine {
public:
  TeleopManipulation(StateId state_id, PinocchioRobotSystem *robot,
                     DracoControlArchitecture *ctrl_arch);
  ~TeleopManipulation();

  void FirstVisit() override;
  void OneStep() override;
  bool EndOfState() override;
  void LastVisit() override;
  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

  Eigen::Isometry3d target_rh_iso_;
  Eigen::Isometry3d target_lh_iso_;
  Eigen::Isometry3d initial_torso_to_rh_iso_;
  Eigen::Isometry3d initial_torso_to_lh_iso_;

private:
  DracoControlArchitecture *ctrl_arch_;

  DracoStateProvider *sp_;
  std::unique_ptr<TeleopHandler> teleop_handler_;

  int teleop_freq_;

  DracoRSCommands *rs_commands_;
  bool b_first_visit_;
  bool b_transition_;
  bool b_teleop_mode_;
  bool b_initialized_;

  double moving_duration_;
  double transition_duration_;

  std::unordered_map<std::string, double> left_gripper_target_pos_;
  std::unordered_map<std::string, double> right_gripper_target_pos_;
  bool b_prev_grasp_ = false;
};

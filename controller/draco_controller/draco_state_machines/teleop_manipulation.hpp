#pragma once

#include "controller/state_machine.hpp"

class PinocchioRobotSystem;
class DracoControlArchitecture;
class DracoStateProvider;
class TeleopHandler;

class TeleopManipulation : public StateMachine {
public:
  TeleopManipulation(StateId state_id, PinocchioRobotSystem *robot,
                     DracoControlArchitecture *ctrl_arch);
  ~TeleopManipulation() = default;

  void FirstVisit() override;
  void OneStep() override;
  bool EndOfState() override;
  void LastVisit() override;
  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

  Eigen::VectorXd target_rh_pos_;
  Eigen::VectorXd target_rh_ori_;

  Eigen::VectorXd target_lh_pos_;
  Eigen::VectorXd target_lh_ori_;

private:
  DracoControlArchitecture *ctrl_arch_;

  DracoStateProvider *sp_;
  std::unique_ptr<TeleopHandler> teleop_handler_;

  bool b_transitted_;
  bool b_initialized_;

  double moving_duration_;
  double transition_duration_;
  double initialization_duration_;

  double transition_time_;
  double transition_start_time_;
};

#pragma once
#include "controller/state_machine.hpp"

class Med7StateProvider;
class Med7ControlArchitecture;
class MinJerkCurveVec;

class StandUp : public StateMachine {
public:
  StandUp(const StateId state_id, PinocchioRobotSystem *robot,
             Med7ControlArchitecture *ctrl_arch);
  ~StandUp();

  void FirstVisit() override;
  void OneStep() override;
  void LastVisit() override;
  bool EndOfState() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

private:
  Med7ControlArchitecture *ctrl_arch_;
  Med7StateProvider *sp_;

  Eigen::VectorXd target_joint_pos_;
  Eigen::VectorXd init_joint_pos_;

  bool b_stay_here_;
  double wait_time_;

  MinJerkCurveVec *min_jerk_curves_;
};

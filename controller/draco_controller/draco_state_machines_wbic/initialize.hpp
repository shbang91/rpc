#pragma once
#include "controller/state_machine.hpp"

class DracoStateProvider;
class DracoControlArchitecture_WBIC;
class MinJerkCurveVec;

class Initialize : public StateMachine {
public:
  Initialize(const StateId state_id, PinocchioRobotSystem *robot,
             DracoControlArchitecture_WBIC *ctrl_arch);
  ~Initialize();

  void FirstVisit() override;
  void OneStep() override;
  void LastVisit() override;
  bool EndOfState() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

private:
  DracoControlArchitecture_WBIC *ctrl_arch_;
  DracoStateProvider *sp_;

  Eigen::VectorXd target_joint_pos_;
  Eigen::VectorXd init_joint_pos_;

  bool b_stay_here_;
  double wait_time_;

  MinJerkCurveVec *min_jerk_curves_;
};

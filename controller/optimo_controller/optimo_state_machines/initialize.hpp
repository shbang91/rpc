#pragma once
#include "controller/state_machine.hpp"

class OptimoStateProvider;
class OptimoControlArchitecture;
class MinJerkCurveVec;

class Initialize : public StateMachine {
public:
  Initialize(const StateId state_id, PinocchioRobotSystem *robot,
             OptimoControlArchitecture *ctrl_arch);
  ~Initialize();

  void FirstVisit() override;
  void OneStep() override;
  void LastVisit() override;
  bool EndOfState() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

private:
  OptimoControlArchitecture *ctrl_arch_;
  OptimoStateProvider *sp_;

  Eigen::VectorXd target_joint_pos_;
  Eigen::VectorXd init_joint_pos_;

  Eigen::Isometry3d init_iso_;
  Eigen::Isometry3d target_iso_;

  bool b_stay_here_;
  double wait_time_;
  double task_transit_time_;

  MinJerkCurveVec *min_jerk_curves_;
};
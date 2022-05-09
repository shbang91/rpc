#pragma once
#include "pnc/state_machine.hpp"
#include <Eigen/Dense>

class FixedDracoStateProvider;
class FixedDracoControlArchitecture;

class Initialize : public StateMachine {
public:
  Initialize(StateId _state_id, FixedDracoControlArchitecture *_ctrl_arch,
             RobotSystem *_robot);
  ~Initialize();

  void FirstVisit();
  void OneStep();
  bool EndOfState();
  void LastVisit();
  StateId GetNextState();

  Eigen::VectorXd des_jpos_;
  double duration_;

private:
  FixedDracoStateProvider *sp_;
  FixedDracoControlArchitecture *ctrl_arch_;
  Eigen::VectorXd ini_jpos_;
};

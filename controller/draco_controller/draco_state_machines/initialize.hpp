#pragma once
#include "controller/state_machine.hpp"

class DracoStateProvider;
class DracoControlArchitecture;
class Initialize : public StateMachine {
public:
  Initialize(const StateId state_id, PinocchioRobotSystem *robot,
             DracoControlArchitecture *ctrl_arch);
  virtual ~Initialize() = default;

  void FirstVisit() override;
  void OneStep() override;
  void LastVisit() override;
  bool EndOfState() override;

  StateId GetNextState() override;

  void InitializeParameters(const YAML::Node &node) override;

private:
  DracoControlArchitecture *ctrl_arch_;
  DracoStateProvider *sp_;
  double duration_;
  Eigen::VectorXd target_joint_pos_;
  Eigen::VectorXd init_joint_pos_;
};

#pragma once

#include "controller/state_machine.hpp"

class DracoControlArchitecture;
class DracoStateProvider;
class FootLandingTransition : public StateMachine {
public:
  FootLandingTransition(const StateId state_id, PinocchioRobotSystem *robot,
                        DracoControlArchitecture *ctrl_arch);
  ~FootLandingTransition() = default;

  void FirstVisit() override;
  void OneStep() override;
  void LastVisit() override;
  bool EndOfState() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

  bool b_static_walking_trigger_;

private:
  DracoControlArchitecture *ctrl_arch_;
  DracoStateProvider *sp_;
};

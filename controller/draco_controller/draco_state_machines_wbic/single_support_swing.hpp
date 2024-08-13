#pragma once

#include "controller/state_machine.hpp"

class PinocchioRobotSystem;
class DracoControlArchitecture_WBIC;
class DracoStateProvider;

class SingleSupportSwing_WBIC : public StateMachine {
public:
  SingleSupportSwing_WBIC(StateId state_id, PinocchioRobotSystem *robot,
                          DracoControlArchitecture_WBIC *ctrl_arch);
  ~SingleSupportSwing_WBIC() = default;

  void FirstVisit() override;
  void OneStep() override;
  void LastVisit() override;
  bool EndOfState() override;
  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

private:
  DracoControlArchitecture_WBIC *ctrl_arch_;
  DracoStateProvider *sp_;

  double swing_height_;
};

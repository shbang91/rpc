#pragma once

#include "controller/state_machine.hpp"

class DracoControlArchitecture;
class DracoStateProvider;
class DoubleSupportStandUp : public StateMachine {
public:
  DoubleSupportStandUp(const StateId state_id, PinocchioRobotSystem *robot,
                       DracoControlArchitecture *ctrl_arch);
  ~DoubleSupportStandUp() = default;

  void FirstVisit() override;
  void OneStep() override;
  void LastVisit() override;
  bool EndOfState() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

private:
  DracoControlArchitecture *ctrl_arch_;
  DracoStateProvider *sp_;

  double target_height_;
  bool b_use_base_height_;

  double rf_z_max_interp_duration_;
};

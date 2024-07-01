#pragma once
#include "controller/state_machine.hpp"

class PinocchioRobotSystem;
class DracoControlArchitecture;
class DracoStateProvider;

class DoubleSupportBalance : public StateMachine {
public:
  DoubleSupportBalance(const StateId state_id, PinocchioRobotSystem *robot,
                       DracoControlArchitecture *ctrl_arch);
  ~DoubleSupportBalance() = default;

  void FirstVisit() override;
  void OneStep() override;
  void LastVisit() override;
  bool EndOfState() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;


private:
  DracoControlArchitecture *ctrl_arch_;

  DracoStateProvider *sp_;

  // set nominal desired position/orientation (e.g., for zero acceleration cmd)
  bool b_use_fixed_foot_pos_;
  Eigen::Isometry3d nominal_lfoot_iso_;
  Eigen::Isometry3d nominal_rfoot_iso_;
};
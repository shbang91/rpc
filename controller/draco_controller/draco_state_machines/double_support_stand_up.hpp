#pragma once

#include "controller/state_machine.hpp"

namespace {
constexpr double kGravity = 9.81;
}

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
  Eigen::Matrix<double, 6, 1> init_reaction_force_;
  Eigen::Matrix<double, 6, 1> des_reaction_force_;
};

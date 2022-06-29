#pragma once
#include "controller/state_machine.hpp"

class PinocchioRobotSystem;
class DracoControlArchitecture;
class DracoStateProvider;

class DoubleSupportSwaying : public StateMachine {
public:
  DoubleSupportSwaying(const StateId state_id, PinocchioRobotSystem *robot,
                       DracoControlArchitecture *ctrl_arch);
  virtual ~DoubleSupportSwaying() = default;

  void FirstVisit() override;
  void OneStep() override;
  bool EndOfState() override;
  void LastVisit() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

private:
  DracoControlArchitecture *ctrl_arch_;
  DracoStateProvider *sp_;

  Eigen::Vector3d amp_;
  Eigen::Vector3d freq_;
};

#pragma once
#include "controller/state_machine.hpp"

class PinocchioRobotSystem;
class DracoControlArchitecture_WBIC;
class DracoStateProvider;

class DoubleSupportSwaying : public StateMachine {
public:
  DoubleSupportSwaying(const StateId state_id, PinocchioRobotSystem *robot,
                       DracoControlArchitecture_WBIC *ctrl_arch);
  ~DoubleSupportSwaying() = default;

  void FirstVisit() override;
  void OneStep() override;
  bool EndOfState() override;
  void LastVisit() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

private:
  DracoControlArchitecture_WBIC *ctrl_arch_;
  DracoStateProvider *sp_;

  Eigen::Vector3d amp_;
  Eigen::Vector3d freq_;
};

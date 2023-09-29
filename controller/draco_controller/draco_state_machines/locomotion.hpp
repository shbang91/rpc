#pragma once
#include "controller/state_machine.hpp"

class PinocchioRobotSystem;
class DracoControlArchitecture;
class DracoStateProvider;
class GaitCommand;

class Locomotion : public StateMachine {
public:
  Locomotion(const StateId state_id, PinocchioRobotSystem *robot,
             DracoControlArchitecture *ctrl_arch);
  virtual ~Locomotion() = default;

  void FirstVisit() override;
  void OneStep() override;
  bool EndOfState() override;
  void LastVisit() override;

  StateId GetNextState() override;

  void SetParameters(const YAML::Node &node) override;

private:
  DracoControlArchitecture *ctrl_arch_;
  DracoStateProvider *sp_;

  std::shared_ptr<GaitCommand> gait_command_;
  double x_vel_cmd_, y_vel_cmd_, yaw_rate_cmd_;
  int gait_number_;
  double swing_height_;
};

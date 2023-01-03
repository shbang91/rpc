#include "controller/draco_state_machines/single_support_swing.hpp"

SingleSupportSwing::SingleSupportSwing(StateId state_id,
                                       PinocchioRobotSystem *robot,
                                       DracoControlArchitecture *ctrl_arch) {}

void SingleSupportSwing::FirstVisit() {}

void SingleSupportSwing::OneStep() {}

void SingleSupportSwing::LastVisit() {}

bool SingleSupportSwing::EndOfState() {
  return state_machine_time_ > end_time_ ? true : false;
}

StateId SingleSupportSwing::GetNextState() {}

void SingleSupportSwing::SetParameters(const YAML::Node &node) {}

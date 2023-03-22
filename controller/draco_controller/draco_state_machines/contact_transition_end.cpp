#include "controller/draco_controller/draco_state_machines/contact_transition_end.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"

ContactTransitionEnd::ContactTransitionEnd(StateId state_id,
                                           PinocchioRobotSystem *robot,
                                           DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch) {

  if (state_id_ == draco_states::kLFContactTransitionEnd)
    util::PrettyConstructor(2, "LFContactTransitionEnd");
  else if (state_id_ == draco_states::kRFContactTransitionEnd)
    util::PrettyConstructor(2, "RFContactTransitionEnd");

  sp_ = DracoStateProvider::GetStateProvider();
}

void ContactTransitionEnd::FirstVisit() {
  state_machine_start_time_ = sp_->current_time_;

  end_time_ =
      ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampDownTime();

  if (state_id_ == draco_states::kLFContactTransitionEnd) {
    std::cout << "draco_States::kLFContactTransitionEnd" << std::endl;
    // =====================================================================
    // task hierarchy manager initialize
    // =====================================================================
    // TODO

    // =====================================================================
    // contact max normal force manager initialize
    // =====================================================================
    ctrl_arch_->lf_max_normal_froce_tm_->InitializeRampToMin(end_time_);
  } else if (state_id_ == draco_states::kRFContactTransitionEnd) {
    std::cout << "draco_states::kRFContactTransitionEnd" << std::endl;
    // =====================================================================
    // task hierarchy manager initialize
    // =====================================================================
    // TODO

    // =====================================================================
    // contact max normal force manager initialize
    // =====================================================================
    ctrl_arch_->rf_max_normal_froce_tm_->InitializeRampToMin(end_time_);
  }
}

void ContactTransitionEnd::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // com pos & torso_ori task update
  ctrl_arch_->dcm_tm_->UpdateDesired(sp_->current_time_);

  // foot pose task update
  ctrl_arch_->lf_SE3_tm_->UseCurrent();
  ctrl_arch_->rf_SE3_tm_->UseCurrent();

  // contact task & force update
  if (state_id_ == draco_states::kLFContactTransitionEnd) {
    ctrl_arch_->lf_max_normal_froce_tm_->UpdateRampToMin(state_machine_time_);
  } else if (state_id_ == draco_states::kRFContactTransitionEnd) {
    ctrl_arch_->rf_max_normal_froce_tm_->UpdateRampToMin(state_machine_time_);
  }
}

bool ContactTransitionEnd::EndOfState() {
  return state_machine_time_ > end_time_ ? true : false;
}

void ContactTransitionEnd::LastVisit() { state_machine_time_ = 0.; }

StateId ContactTransitionEnd::GetNextState() {
  if (state_id_ == draco_states::kLFContactTransitionEnd)
    return draco_states::kLFSingleSupportSwing;
  else if (state_id_ == draco_states::kRFContactTransitionEnd)
    return draco_states::kRFSingleSupportSwing;
}

void ContactTransitionEnd::SetParameters(const YAML::Node &node) {}

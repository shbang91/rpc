#include "controller/draco_controller/draco_state_machines/contact_transition_start.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"

ContactTransitionStart::ContactTransitionStart(
    StateId state_id, PinocchioRobotSystem *robot,
    DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch), end_time_(0.) {

  state_id_ == draco_states::kLFContactTransitionStart
      ? util::PrettyConstructor(2, "LFContactTransitionStart")
      : util::PrettyConstructor(2, "RFContactTransitionStart");

  sp_ = DracoStateProvider::GetStateProvider();
}

void ContactTransitionStart::FirstVisit() {
  state_machine_start_time_ = sp_->current_time_;

  if (state_id_ == draco_states::kLFContactTransitionStart) {

    std::cout << "kLFContactTransitionStart" << std::endl;
    // =====================================================================
    // task hierarchy manager initialize
    // =====================================================================
    ctrl_arch_->rf_pos_hm_->InitializeRampToMax(
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());
    ctrl_arch_->rf_ori_hm_->InitializeRampToMax(
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());

    // =====================================================================
    // contact max normal force manager initialize
    // =====================================================================
    ctrl_arch_->rf_max_normal_froce_tm_->InitializeRampToMax(
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());
  } else {

    std::cout << "kRFContactTransitionStart" << std::endl;
    // =====================================================================
    // task hierarchy manager initialize
    // =====================================================================
    ctrl_arch_->lf_pos_hm_->InitializeRampToMax(
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());
    ctrl_arch_->lf_ori_hm_->InitializeRampToMax(
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());
    // =====================================================================
    // contact max normal force manager initialize
    // =====================================================================
    ctrl_arch_->lf_max_normal_froce_tm_->InitializeRampToMax(
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());
  }

  // =====================================================================
  // dcm planner initialize
  // =====================================================================
  if (ctrl_arch_->dcm_tm_->NoRemainingSteps())
    end_time_ =
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetFinalContactTransferTime();
  else {
    Eigen::Quaterniond init_torso_quat(
        robot_->GetLinkIsometry(draco_link::torso_com_link).linear());
    Eigen::Vector3d init_dcm_pos = sp_->dcm_;
    Eigen::Vector3d init_dcm_vel = sp_->dcm_vel_;
    if (sp_->b_use_base_height_) {
      init_dcm_pos[2] =
          robot_->GetLinkIsometry(draco_link::torso_com_link).translation()[2];
      init_dcm_vel[2] =
          robot_->GetLinkSpatialVel(draco_link::torso_com_link)[5];
    }

    if (sp_->prev_state_ == draco_states::kDoubleSupportBalance) {
      // TODO: consider torso ang vel
      ctrl_arch_->dcm_tm_->Initialize(
          sp_->current_time_, dcm_transfer_type::kInitial, init_torso_quat,
          init_dcm_pos, init_dcm_vel);
      end_time_ =
          ctrl_arch_->dcm_tm_->GetDCMPlanner()
              ->GetInitialContactTransferTime() -
          ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampDownTime();
    }
    // else {
    // TODO: replanning
    // TODO: consider torso ang vel
    // ctrl_arch_->dcm_tm_->Initialize(
    // sp_->current_time_, dcm_transfer_type::kMidStep, init_torso_quat,
    // init_dcm_pos, init_dcm_vel);
    // end_time_ =
    // ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime();
    //}
  }
}

void ContactTransitionStart::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // com pos & torso_ori task update
  ctrl_arch_->dcm_tm_->UpdateDesired(sp_->current_time_);

  // foot pose task update
  ctrl_arch_->lf_SE3_tm_->UseCurrent();
  ctrl_arch_->rf_SE3_tm_->UseCurrent();

  // contact max normal force & foot task hierarchy update
  if (state_id_ == draco_states::kLFContactTransitionStart) {
    ctrl_arch_->rf_max_normal_froce_tm_->UpdateRampToMax(state_machine_time_);
    ctrl_arch_->rf_pos_hm_->UpdateRampToMax(state_machine_time_);
    ctrl_arch_->rf_ori_hm_->UpdateRampToMax(state_machine_time_);
  } else {
    ctrl_arch_->lf_max_normal_froce_tm_->UpdateRampToMax(state_machine_time_);
    ctrl_arch_->lf_pos_hm_->UpdateRampToMax(state_machine_time_);
    ctrl_arch_->lf_ori_hm_->UpdateRampToMax(state_machine_time_);
  }
}

bool ContactTransitionStart::EndOfState() {
  return state_machine_time_ > end_time_ ? true : false;
}

void ContactTransitionStart::LastVisit() {}

StateId ContactTransitionStart::GetNextState() {
  if (ctrl_arch_->dcm_tm_->NoRemainingSteps())
    return draco_states::kDoubleSupportBalance;
  else {
    if (state_id_ == draco_states::kLFContactTransitionStart) {
      sp_->stance_foot_ = draco_link::r_foot_contact;
      return draco_states::kLFContactTransitionEnd;

    } else {
      sp_->stance_foot_ = draco_link::l_foot_contact;
      return draco_states::kRFContactTransitionEnd;
    }
  }
}

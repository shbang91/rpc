#include "controller/draco_controller/draco_state_machines/foot_landing_transition.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "util/util.hpp"

FootLandingTransition::FootLandingTransition(
    const StateId state_id, PinocchioRobotSystem *robot,
    DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch),
      b_static_walking_trigger_(false) {
  util::PrettyConstructor(2, "FootLandingTransition");

  sp_ = DracoStateProvider::GetStateProvider();
}

void FootLandingTransition::FirstVisit() {
  state_machine_start_time_ = sp_->current_time_;

  sp_->b_lf_contact_ = true;
  sp_->b_rf_contact_ = true;

  if (sp_->stance_foot_ == draco_link::l_foot_contact) {
    std::cout << "draco_states::kRFootLandingTransition" << std::endl;

    // reaction force manager
    ctrl_arch_->rf_max_normal_froce_tm_->InitializeRampToMax(end_time_);
    ctrl_arch_->lf_max_normal_froce_tm_->InitializeRampToMax(end_time_);

    // task hierarchy manager
    ctrl_arch_->rf_pos_hm_->InitializeRampToMax(end_time_);
    ctrl_arch_->lf_pos_hm_->InitializeRampToMax(end_time_);
    ctrl_arch_->rf_ori_hm_->InitializeRampToMax(end_time_);
    ctrl_arch_->lf_ori_hm_->InitializeRampToMax(end_time_);

  } else if (sp_->stance_foot_ == draco_link::r_foot_contact) {
    std::cout << "draco_states::kLFootLandingTransition" << std::endl;
    // reaction force manager
    ctrl_arch_->lf_max_normal_froce_tm_->InitializeRampToMax(end_time_);
    ctrl_arch_->rf_max_normal_froce_tm_->InitializeRampToMax(end_time_);

    // task hierarchy manager
    ctrl_arch_->lf_pos_hm_->InitializeRampToMax(end_time_);
    ctrl_arch_->rf_pos_hm_->InitializeRampToMax(end_time_);
    ctrl_arch_->lf_ori_hm_->InitializeRampToMax(end_time_);
    ctrl_arch_->rf_ori_hm_->InitializeRampToMax(end_time_);

  } else {
    assert(false);
  }
}

void FootLandingTransition::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  if (sp_->stance_foot_ == draco_link::l_foot_contact) {
    // reaction force manager
    ctrl_arch_->rf_max_normal_froce_tm_->UpdateRampToMax(end_time_);
    ctrl_arch_->lf_max_normal_froce_tm_->UpdateRampToMax(end_time_);

    // task hierarchy manager
    ctrl_arch_->rf_pos_hm_->UpdateRampToMax(end_time_);
    ctrl_arch_->lf_pos_hm_->UpdateRampToMax(end_time_);
    ctrl_arch_->rf_ori_hm_->UpdateRampToMax(end_time_);
    ctrl_arch_->lf_ori_hm_->UpdateRampToMax(end_time_);

  } else if (sp_->stance_foot_ == draco_link::r_foot_contact) {
    // reaction force manager
    ctrl_arch_->lf_max_normal_froce_tm_->UpdateRampToMax(end_time_);
    ctrl_arch_->rf_max_normal_froce_tm_->UpdateRampToMax(end_time_);

    // task hierarchy manager
    ctrl_arch_->lf_pos_hm_->UpdateRampToMax(end_time_);
    ctrl_arch_->rf_pos_hm_->UpdateRampToMax(end_time_);
    ctrl_arch_->lf_ori_hm_->UpdateRampToMax(end_time_);
    ctrl_arch_->rf_ori_hm_->UpdateRampToMax(end_time_);

  } else {
    assert(false);
  }

  // foot task
  ctrl_arch_->lf_SE3_tm_->UseCurrent();
  ctrl_arch_->rf_SE3_tm_->UseCurrent();
}

void FootLandingTransition::LastVisit() {
  b_static_walking_trigger_ = false;
  if (state_id_ == draco_states::kLFootLandingTransition)
    sp_->stance_foot_ = draco_link::l_foot_contact;
  if (state_id_ == draco_states::kRFootLandingTransition)
    sp_->stance_foot_ = draco_link::r_foot_contact;
}

bool FootLandingTransition::EndOfState() {
  return (state_machine_time_ > end_time_ && b_static_walking_trigger_) ? true
                                                                        : false;
}

StateId FootLandingTransition::GetNextState() {
  return draco_states::kDoubleSupportMoveCoMCenter;
}

void FootLandingTransition::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "max_rf_z_ramp_time", end_time_);
  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

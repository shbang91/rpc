#include "controller/draco_controller/draco_state_machines/foot_lifting_transition.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "util/util.hpp"

FootLiftingTransition::FootLiftingTransition(
    const StateId state_id, PinocchioRobotSystem *robot,
    DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch),
      b_static_walking_trigger_(false) {
  util::PrettyConstructor(2, "FootLiftingTransition");

  sp_ = DracoStateProvider::GetStateProvider();
}

void FootLiftingTransition::FirstVisit() {
  state_machine_start_time_ = sp_->current_time_;

  sp_->b_lf_contact_ = true;
  sp_->b_rf_contact_ = true;

  if (sp_->stance_foot_ == draco_link::l_foot_contact) {
    std::cout << "draco_states::kRFootLiftingTransition" << std::endl;

    // right foot
    sp_->nominal_right_foot_iso_.translation() =
        ctrl_arch_->tci_container_->task_map_["rf_pos_task"]->DesiredPos();
    Eigen::VectorXd rf_quat_vec =
        ctrl_arch_->tci_container_->task_map_["rf_ori_task"]->DesiredPos();
    Eigen::Quaterniond rf_quat(rf_quat_vec[3], rf_quat_vec[0], rf_quat_vec[1],
                               rf_quat_vec[2]);
    sp_->nominal_right_foot_iso_.linear() =
        rf_quat.normalized().toRotationMatrix();

    // reaction force manager
    ctrl_arch_->rf_max_normal_froce_tm_->InitializeRampToMin(end_time_);
    ctrl_arch_->lf_max_normal_froce_tm_->InitializeRampToMax(end_time_);

    // task hierarchy manager
    ctrl_arch_->rf_pos_hm_->InitializeRampToMin(end_time_);
    ctrl_arch_->lf_pos_hm_->InitializeRampToMax(end_time_);
    ctrl_arch_->rf_ori_hm_->InitializeRampToMin(end_time_);
    ctrl_arch_->lf_ori_hm_->InitializeRampToMax(end_time_);

  } else if (sp_->stance_foot_ == draco_link::r_foot_contact) {
    std::cout << "draco_states::kLFootLiftingTransition" << std::endl;

    // left foot
    sp_->nominal_left_foot_iso_.translation() =
        ctrl_arch_->tci_container_->task_map_["lf_pos_task"]->DesiredPos();
    Eigen::VectorXd lf_quat_vec =
        ctrl_arch_->tci_container_->task_map_["lf_ori_task"]->DesiredPos();
    Eigen::Quaterniond lf_quat(lf_quat_vec[3], lf_quat_vec[0], lf_quat_vec[1],
                               lf_quat_vec[2]);
    sp_->nominal_left_foot_iso_.linear() =
        lf_quat.normalized().toRotationMatrix();

    // reaction force manager
    ctrl_arch_->lf_max_normal_froce_tm_->InitializeRampToMin(end_time_);
    ctrl_arch_->rf_max_normal_froce_tm_->InitializeRampToMax(end_time_);

    // task hierarchy manager
    ctrl_arch_->lf_pos_hm_->InitializeRampToMin(end_time_);
    ctrl_arch_->rf_pos_hm_->InitializeRampToMax(end_time_);
    ctrl_arch_->lf_ori_hm_->InitializeRampToMin(end_time_);
    ctrl_arch_->rf_ori_hm_->InitializeRampToMax(end_time_);

  } else {
    assert(false);
  }
}

void FootLiftingTransition::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  if (sp_->stance_foot_ == draco_link::l_foot_contact) {
    // reaction force manager
    ctrl_arch_->rf_max_normal_froce_tm_->UpdateRampToMin(end_time_);
    ctrl_arch_->lf_max_normal_froce_tm_->UpdateRampToMax(end_time_);

    // task hierarchy manager
    ctrl_arch_->rf_pos_hm_->UpdateRampToMin(end_time_);
    ctrl_arch_->lf_pos_hm_->UpdateRampToMax(end_time_);
    ctrl_arch_->rf_ori_hm_->UpdateRampToMin(end_time_);
    ctrl_arch_->lf_ori_hm_->UpdateRampToMax(end_time_);

  } else if (sp_->stance_foot_ == draco_link::r_foot_contact) {
    // reaction force manager
    ctrl_arch_->lf_max_normal_froce_tm_->UpdateRampToMin(end_time_);
    ctrl_arch_->rf_max_normal_froce_tm_->UpdateRampToMax(end_time_);

    // task hierarchy manager
    ctrl_arch_->lf_pos_hm_->UpdateRampToMin(end_time_);
    ctrl_arch_->rf_pos_hm_->UpdateRampToMax(end_time_);
    ctrl_arch_->lf_ori_hm_->UpdateRampToMin(end_time_);
    ctrl_arch_->rf_ori_hm_->UpdateRampToMax(end_time_);

  } else {
    assert(false);
  }

  // foot task
  // ctrl_arch_->lf_SE3_tm_->UseCurrent();
  // ctrl_arch_->rf_SE3_tm_->UseCurrent();
  ctrl_arch_->lf_SE3_tm_->UseNominal(sp_->nominal_left_foot_iso_);
  ctrl_arch_->rf_SE3_tm_->UseNominal(sp_->nominal_right_foot_iso_);
}

void FootLiftingTransition::LastVisit() { b_static_walking_trigger_ = false; }

bool FootLiftingTransition::EndOfState() {
  // return (state_machine_time_ > end_time_ && b_static_walking_trigger_) ?
  // true
  return (state_machine_time_ > end_time_) ? true : false;
}

StateId FootLiftingTransition::GetNextState() {
  if (state_id_ == draco_states::kLFootLiftingTransition)
    return draco_states::kLFootLifting;
  if (state_id_ == draco_states::kRFootLiftingTransition)
    return draco_states::kRFootLifting;
}

void FootLiftingTransition::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "max_rf_z_ramp_time", end_time_);
  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

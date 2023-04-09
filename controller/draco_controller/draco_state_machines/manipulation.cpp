#include "controller/draco_controller/draco_state_machines/manipulation.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/hand_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "controller/whole_body_controller/task.hpp"

Manipulation::Manipulation(StateId state_id, PinocchioRobotSystem *robot,
                           DracoControlArchitecture *ctrl_arch)
    : Background(state_id, robot), ctrl_arch_(ctrl_arch) {
  util::PrettyConstructor(2, "BackgroundManipulation");
  sp_ = DracoStateProvider::GetStateProvider();
  std::cout << "Manipulation Constructed" << std::endl;

  target_rh_pos_ = Eigen::VectorXd::Zero(3); // TODO: make 0 0 0
  target_rh_pos_ << 0.3, -0.3, 0.3;

  target_rh_ori_ = Eigen::VectorXd::Zero(4);
  target_rh_ori_ << 0, -0.707, 0, 0.707;

  target_lh_pos_ = Eigen::VectorXd::Zero(3); // TODO: make 0 0 0
  target_lh_pos_ << 0.3, 0.3, 0.3;

  target_lh_ori_ = Eigen::VectorXd::Zero(4);
  target_lh_ori_ << 0, -0.707, 0, 0.707;

  background_time_ = 0.;
  moving_duration_ = 0.05;

  initialized_ = 0;
  initialization_duration_ = 0.2;

  transitted_ = 0;
  transition_duration_ = 0.3;

  ctrl_arch_->lh_pos_hm_->InitializeRampToMin(transition_duration_);
  ctrl_arch_->lh_ori_hm_->InitializeRampToMin(transition_duration_);
  ctrl_arch_->rh_pos_hm_->InitializeRampToMin(transition_duration_);
  ctrl_arch_->rh_ori_hm_->InitializeRampToMin(transition_duration_);

  ctrl_arch_->lh_pos_hm_->UpdateRampToMin(transition_duration_);
  ctrl_arch_->lh_ori_hm_->UpdateRampToMin(transition_duration_);
  ctrl_arch_->rh_pos_hm_->UpdateRampToMin(transition_duration_);
  ctrl_arch_->rh_ori_hm_->UpdateRampToMin(transition_duration_);
}

void Manipulation::FirstVisit() {
  background_start_time_ = sp_->current_time_;

  Eigen::Isometry3d target_rh_iso;
  Eigen::Isometry3d target_lh_iso;
  Eigen::Quaterniond target_rh_quat;
  Eigen::Quaterniond target_lh_quat;

  target_rh_iso.translation() = target_rh_pos_;
  target_rh_quat.x() = target_rh_ori_(0);
  target_rh_quat.y() = target_rh_ori_(1);
  target_rh_quat.z() = target_rh_ori_(2);
  target_rh_quat.w() = target_rh_ori_(3);
  target_rh_quat = target_rh_quat.normalized();
  target_rh_iso.linear() = target_rh_quat.toRotationMatrix();

  target_lh_iso.translation() = target_lh_pos_;
  target_lh_quat.x() = target_lh_ori_(0);
  target_lh_quat.y() = target_lh_ori_(1);
  target_lh_quat.z() = target_lh_ori_(2);
  target_lh_quat.w() = target_lh_ori_(3);
  target_lh_quat = target_lh_quat.normalized();
  target_lh_iso.linear() = target_lh_quat.toRotationMatrix();

  ctrl_arch_->rh_SE3_tm_->InitializeHandTrajectory(
      target_rh_iso, 0.0, moving_duration_, initialized_);
  ctrl_arch_->lh_SE3_tm_->InitializeHandTrajectory(
      target_lh_iso, 0.0, moving_duration_, initialized_);

  if (!initialized_) {
    ctrl_arch_->rh_SE3_tm_->InitializeHandTrajectory(
        target_rh_iso, 0.0, initialization_duration_, initialized_);
    ctrl_arch_->lh_SE3_tm_->InitializeHandTrajectory(
        target_lh_iso, 0.0, initialization_duration_, initialized_);
  }

  if (!transitted_) {
    transition_start_time_ = sp_->current_time_;
    if (state_id_ == draco_states::kDHManipulation) {
      ctrl_arch_->lh_pos_hm_->InitializeRampToMax(transition_duration_);
      ctrl_arch_->lh_ori_hm_->InitializeRampToMax(transition_duration_);
      ctrl_arch_->rh_pos_hm_->InitializeRampToMax(transition_duration_);
      ctrl_arch_->rh_ori_hm_->InitializeRampToMax(transition_duration_);
    } else {
      ctrl_arch_->lh_pos_hm_->InitializeRampToMin(transition_duration_);
      ctrl_arch_->lh_ori_hm_->InitializeRampToMin(transition_duration_);
      ctrl_arch_->rh_pos_hm_->InitializeRampToMin(transition_duration_);
      ctrl_arch_->rh_ori_hm_->InitializeRampToMin(transition_duration_);
    }
  }
}

void Manipulation::OneStep() {

  background_time_ = sp_->current_time_ - background_start_time_;

  if (!transitted_)
    transition_time_ = sp_->current_time_ - transition_start_time_;

  if (state_id_ == draco_states::kDHManipulation) {
    ctrl_arch_->lh_pos_hm_->UpdateRampToMax(transition_time_);
    ctrl_arch_->lh_ori_hm_->UpdateRampToMax(transition_time_);
    ctrl_arch_->rh_pos_hm_->UpdateRampToMax(transition_time_);
    ctrl_arch_->rh_ori_hm_->UpdateRampToMax(transition_time_);
  } else {
    ctrl_arch_->lh_pos_hm_->UpdateRampToMin(transition_time_);
    ctrl_arch_->lh_ori_hm_->UpdateRampToMin(transition_time_);
    ctrl_arch_->rh_pos_hm_->UpdateRampToMin(transition_time_);
    ctrl_arch_->rh_ori_hm_->UpdateRampToMin(transition_time_);
  }

  ctrl_arch_->lh_SE3_tm_->UpdateHandPose(background_time_);
  ctrl_arch_->rh_SE3_tm_->UpdateHandPose(background_time_);
}

bool Manipulation::EndOfState() {
  if (!initialized_)
    initialized_ = background_time_ > initialization_duration_;

  if (!transitted_)
    transitted_ = transition_time_ > transition_duration_;

  return (background_time_ > moving_duration_) and initialized_;
}

void Manipulation::LastVisit() { background_time_ = 0.; }

StateId Manipulation::GetNextState() { return draco_states::kDHManipulation; }

void Manipulation::SetParameters(const YAML::Node &node) {}

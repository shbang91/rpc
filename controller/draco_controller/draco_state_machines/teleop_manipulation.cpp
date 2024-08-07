#include "controller/draco_controller/draco_state_machines/teleop_manipulation.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_rs_teleop_handler.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/hand_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "controller/whole_body_controller/task.hpp"

TeleopManipulation::TeleopManipulation(StateId state_id,
                                       PinocchioRobotSystem *robot,
                                       DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch) {
  util::PrettyConstructor(2, "TeleopManipulation");
  sp_ = DracoStateProvider::GetStateProvider();

  try {
    YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

    // construct teleop handler
    teleop_handler_ = std::make_unique<DracoRSTeleopHandler>();
    std::cout << "Connecting to Teleop Socket...." << std::endl;
    const std::string ip_address =
        util::ReadParameter<std::string>(cfg, "teleop_ip_address");
    if (teleop_handler_->InitializeSocket(ip_address))
      std::cout << "Connected to Teleop Socket!" << std::endl;

  } catch (const std::runtime_error &ex) {
    std::cerr << "Error Reading Parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }

  target_rh_pos_ = Eigen::VectorXd::Zero(3);
  target_rh_pos_ << 0.3, -0.3, 0.3;

  target_rh_ori_ = Eigen::VectorXd::Zero(4);
  target_rh_ori_ << 0, -0.707, 0, 0.707;

  target_lh_pos_ = Eigen::VectorXd::Zero(3);
  target_lh_pos_ << 0.3, 0.3, 0.3;

  target_lh_ori_ = Eigen::VectorXd::Zero(4);
  target_lh_ori_ << 0, -0.707, 0, 0.707;

  moving_duration_ = 0.05;

  b_initialized_ = false;
  initialization_duration_ = 5.0;

  b_transitted_ = false;
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

void TeleopManipulation::FirstVisit() {
  state_machine_start_time_ = sp_->current_time_;

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
      target_rh_iso, 0.0, moving_duration_, b_initialized_);
  ctrl_arch_->lh_SE3_tm_->InitializeHandTrajectory(
      target_lh_iso, 0.0, moving_duration_, b_initialized_);

  if (!b_initialized_) {
    ctrl_arch_->rh_SE3_tm_->InitializeHandTrajectory(
        target_rh_iso, 0.0, initialization_duration_, b_initialized_);
    ctrl_arch_->lh_SE3_tm_->InitializeHandTrajectory(
        target_lh_iso, 0.0, initialization_duration_, b_initialized_);
  }

  if (!b_transitted_) {
    transition_start_time_ = sp_->current_time_;
    if (state_id_ == draco_states::kTeleopManipulation) {
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

void TeleopManipulation::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  if (!b_transitted_)
    transition_time_ = sp_->current_time_ - transition_start_time_;

  if (state_id_ == draco_states::kTeleopManipulation) {
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

  ctrl_arch_->lh_SE3_tm_->UpdateHandPose(state_machine_time_);
  ctrl_arch_->rh_SE3_tm_->UpdateHandPose(state_machine_time_);

  if (!b_initialized_)
    b_initialized_ = state_machine_time_ > initialization_duration_;

  if (!b_transitted_)
    b_transitted_ = transition_time_ > transition_duration_;
}

bool TeleopManipulation::EndOfState() {
  return (state_machine_time_ > moving_duration_) && b_initialized_;
}

void TeleopManipulation::LastVisit() { state_machine_time_ = 0.; }

StateId TeleopManipulation::GetNextState() {
  return draco_states::kTeleopManipulation;
}

void TeleopManipulation::SetParameters(const YAML::Node &node) {}

#include "controller/draco_controller/draco_state_machines/double_support_balance.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/task.hpp"

DoubleSupportBalance::DoubleSupportBalance(const StateId state_id,
                                           PinocchioRobotSystem *robot,
                                           DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch),
      b_com_swaying_(false), b_lmpc_swaying_(false), b_nmpc_swaying_(false),
      b_dcm_walking_(false), b_lmpc_walking_(false), b_nmpc_walking_(false),
      b_static_walking_(false) {
  util::PrettyConstructor(2, "DoubleSupportBalance");

  sp_ = DracoStateProvider::GetStateProvider();
}

void DoubleSupportBalance::FirstVisit() {
  std::cout << "draco_states: kDoubleSupportBalance" << std::endl;
  state_machine_start_time_ = sp_->current_time_;

  // reset flags
  b_com_swaying_ = false;
  b_lmpc_swaying_ = false;
  b_nmpc_swaying_ = false;

  b_dcm_walking_ = false;
  b_lmpc_walking_ = false;
  b_nmpc_walking_ = false;

  b_static_walking_ = false;
}

void DoubleSupportBalance::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // update foot pose task
  // ctrl_arch_->lf_SE3_tm_->UseCurrent();
  // ctrl_arch_->rf_SE3_tm_->UseCurrent();
}

bool DoubleSupportBalance::EndOfState() {
  if (b_com_swaying_ || b_lmpc_swaying_ || b_nmpc_swaying_ || b_lmpc_walking_ ||
      b_nmpc_walking_ || b_static_walking_)
    return true;

  if (b_dcm_walking_ && ctrl_arch_->dcm_tm_->GetFootStepList().size() > 0 &&
      !ctrl_arch_->dcm_tm_->NoRemainingSteps())
    return true;

  return false;
}

void DoubleSupportBalance::LastVisit() {
  state_machine_time_ = 0.;

  if (sp_->b_use_base_height_)
    sp_->des_com_height_ = robot_->GetRobotComPos()[2];

  std::cout << "-----------------------------------------" << std::endl;
  std::cout << "des com height: " << sp_->des_com_height_ << std::endl;
  std::cout << "-----------------------------------------" << std::endl;

  Eigen::Isometry3d torso_iso =
      robot_->GetLinkIsometry(draco_link::torso_com_link);
  FootStep::MakeHorizontal(torso_iso);
  sp_->rot_world_local_ = torso_iso.linear();

  // update foot desried
  Eigen::Isometry3d lf_iso =
      robot_->GetLinkIsometry(draco_link::l_foot_contact);
  Eigen::Isometry3d rf_iso =
      robot_->GetLinkIsometry(draco_link::r_foot_contact);

  ctrl_arch_->tci_container_->task_map_["lf_pos_task"]->UpdateDesired(
      lf_iso.translation(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  ctrl_arch_->tci_container_->task_map_["lf_ori_task"]->UpdateDesired(
      Eigen::Quaterniond(lf_iso.linear()).normalized().coeffs(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  ctrl_arch_->tci_container_->task_map_["rf_pos_task"]->UpdateDesired(
      rf_iso.translation(), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  ctrl_arch_->tci_container_->task_map_["rf_ori_task"]->UpdateDesired(
      Eigen::Quaterniond(rf_iso.linear()).normalized().coeffs(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  // TEST
  // std::cout << "nominal inertia: " << std::endl;
  // std::cout << robot_->GetIg() << std::endl;
  // std::cout << "nominal inertia local: " << std::endl;
  // Eigen::Matrix3d local_inertia = robot_->GetBodyOriRot().transpose() *
  // robot_->GetIg().topLeftCorner<3, 3>() *
  // robot_->GetBodyOriRot();
  // std::cout << local_inertia << std::endl;
  // exit(0);
}

StateId DoubleSupportBalance::GetNextState() {
  if (b_com_swaying_)
    return draco_states::kDoubleSupportSwaying;
  // if (b_lmpc_swaying_)
  // return draco_states::kComSwayingLmpc;
  // if (b_nmpc_swaying_)
  // return draco_states::kComSwayingNmpc;

  if (b_dcm_walking_) {
    if (ctrl_arch_->dcm_tm_->GetSwingLeg() == end_effector::LFoot) {
      b_dcm_walking_ = false;
      return draco_states::kLFContactTransitionStart;
    } else if (ctrl_arch_->dcm_tm_->GetSwingLeg() == end_effector::RFoot) {
      b_dcm_walking_ = false;
      return draco_states::kRFContactTransitionStart;
    }
  }

  //}
  //}

  if (b_lmpc_walking_) {
    b_lmpc_walking_ = false;
    return draco_states::kLocomotion;
  }
  // if (b_nmpc_walking_)
  // return;

  // if (b_static_walking_) {
  // if (true) {
  // return draco_states::kMoveComToLFoot;
  //} else {
  // return draco_states::kMoveComToRFoot;
  //}
  //}
}

void DoubleSupportBalance::SetParameters(const YAML::Node &node) {}

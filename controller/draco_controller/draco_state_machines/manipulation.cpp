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

Manipulation::Manipulation
(StateId state_id, PinocchioRobotSystem *robot, DracoControlArchitecture *ctrl_arch)
: Background(state_id, robot), ctrl_arch_(ctrl_arch) 
{
  util::PrettyConstructor(2, "BackgroundManipulation");
  sp_ = DracoStateProvider::GetStateProvider();

  target_rh_pos_ = Eigen::VectorXd::Zero(3);
  target_rh_ori_ = Eigen::VectorXd::Zero(4);

  target_lh_pos_ = Eigen::VectorXd::Zero(3);
  target_lh_ori_ = Eigen::VectorXd::Zero(4);

  moving_duration_ = 0.0;
  trans_duration_ = 0.0;

  background_time_ = 0.;

}

void Manipulation::FirstVisit() 
{
  background_start_time_ = sp_->current_time_;

  std::cout << "draco_background::kDHManipulation" << std::endl;

  Eigen::Isometry3d target_rh_iso;
  Eigen::Isometry3d target_lh_iso;

  target_rh_iso.translation() = target_rh_pos_;
  target_rh_iso.linear() = target_rh_ori_.normalized().toRotationMatrix();

  target_lh_iso.translation() = target_lh_pos_;
  target_lh_iso.linear() = target_lh_ori_.normalized().toRotationMatrix();

  ctrl_arch_->rh_SE3_tm_->InitializeHandTrajectory(target_rh_iso, background_start_time_, moving_duration_);
  ctrl_arch_->lh_SE3_tm_->InitializeHandTrajectory(target_lh_iso, background_start_time_, moving_duration_);

  if (state_id_ == draco_states::kDoubleSupportBalance) 
  {
    ctrl_arch_->lh_pos_hm_->InitializeRampToMax(trans_duration_);
    ctrl_arch_->lh_ori_hm_->InitializeRampToMax(trans_duration_);
    ctrl_arch_->rh_pos_hm_->InitializeRampToMax(trans_duration_);
    ctrl_arch_->rh_ori_hm_->InitializeRampToMax(trans_duration_);
  }
  else 
  {
    ctrl_arch_->lh_pos_hm_->InitializeRampToMin(trans_duration_);
    ctrl_arch_->lh_ori_hm_->InitializeRampToMin(trans_duration_);
    ctrl_arch_->rh_pos_hm_->InitializeRampToMin(trans_duration_);
    ctrl_arch_->rh_ori_hm_->InitializeRampToMin(trans_duration_);
  }
}

void Manipulation::OneStep() 
{
  background_time_ = sp_->current_time_ - background_start_time_;

  if (state_id_ == draco_states::kDoubleSupportBalance) 
  {
    ctrl_arch_->lh_pos_hm_->UpdateRampToMax(background_start_time_);
    ctrl_arch_->lh_ori_hm_->UpdateRampToMax(background_start_time_);
    ctrl_arch_->rh_pos_hm_->UpdateRampToMax(background_start_time_);
    ctrl_arch_->rh_ori_hm_->UpdateRampToMax(background_start_time_);
  } 
  else 
  {
    ctrl_arch_->lh_pos_hm_->UpdateRampToMax(background_start_time_);
    ctrl_arch_->lh_ori_hm_->UpdateRampToMax(background_start_time_);
    ctrl_arch_->rh_pos_hm_->UpdateRampToMax(background_start_time_);
    ctrl_arch_->rh_ori_hm_->UpdateRampToMax(background_start_time_);
  }

  ctrl_arch_->lh_SE3_tm_->UpdateHandPose(sp_->current_time_);
  ctrl_arch_->rh_SE3_tm_->UpdateHandPose(sp_->current_time_);
}

bool Manipulation::EndOfState() 
{
  // return state_machine_time_ > end_time_ ? true : false;
  return background_time_ > moving_duration_ + 0.1 ? true : false;
}

void Manipulation::LastVisit() 
{ background_time_ = 0.; }

StateId Manipulation::GetNextState() 
{
  return draco_states::kDoubleSupportBalance;
}

void Manipulation::SetParameters(const YAML::Node &node) 
{}

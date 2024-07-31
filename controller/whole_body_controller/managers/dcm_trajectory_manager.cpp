#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"

DCMTrajectoryManager::DCMTrajectoryManager(DCMPlanner *dcm_planner,
                                           Task *com_xy_task, Task *com_z_task,
                                           Task *torso_ori_task,
                                           PinocchioRobotSystem *robot,
                                           const int lfoot_idx,
                                           const int rfoot_idx)
    : dcm_planner_(dcm_planner), com_xy_task_(com_xy_task),
      com_z_task_(com_z_task), torso_ori_task_(torso_ori_task), robot_(robot),
      lfoot_idx_(lfoot_idx), rfoot_idx_(rfoot_idx), current_foot_step_idx_(0),
      first_swing_leg_(end_effector::LFoot), /*b_first_visit_(true),*/
      walking_primitive_(-1) {
  util::PrettyConstructor(2, "DCMTrajectoryManager");

  foot_step_list_.clear();
  foot_step_preview_list_.clear();
}

DCMTrajectoryManager::DCMTrajectoryManager(
    DCMPlanner *dcm_planner, Task *com_xy_task, Task *com_z_task,
    Task *torso_ori_task, PinocchioRobotSystem *robot, const int lfoot_idx,
    const int rfoot_idx, bool b_use_base_height)
    : dcm_planner_(dcm_planner), com_xy_task_(com_xy_task),
      com_z_task_(com_z_task), torso_ori_task_(torso_ori_task), robot_(robot),
      lfoot_idx_(lfoot_idx), rfoot_idx_(rfoot_idx), current_foot_step_idx_(0),
      first_swing_leg_(end_effector::LFoot), /*b_first_visit_(true),*/
      walking_primitive_(-1), b_use_base_height_(b_use_base_height) {
  util::PrettyConstructor(2, "DCMTrajectoryManager");

  foot_step_list_.clear();
  foot_step_preview_list_.clear();
}

void DCMTrajectoryManager::GenerateFootSteps() {
  //---------------------------------------------------------
  // foot step setup
  //---------------------------------------------------------
  // update current stance feet
  Eigen::Isometry3d lfoot_iso = robot_->GetLinkIsometry(lfoot_idx_);
  FootStep::MakeHorizontal(lfoot_iso);
  init_left_foot_.SetPosOriSide(lfoot_iso.translation(),
                                Eigen::Quaterniond(lfoot_iso.linear()),
                                end_effector::LFoot);

  Eigen::Isometry3d rfoot_iso = robot_->GetLinkIsometry(rfoot_idx_);
  FootStep::MakeHorizontal(rfoot_iso);
  init_right_foot_.SetPosOriSide(rfoot_iso.translation(),
                                 Eigen::Quaterniond(rfoot_iso.linear()),
                                 end_effector::RFoot);

  FootStep init_mid_foot;
  FootStep::ComputeMidFoot(init_left_foot_, init_right_foot_, init_mid_foot);

  // generate foot step list depening on walking primitives
  switch (walking_primitive_) {
  case dcm_walking_primitive::kFwdWalk:
    _ResetIndexAndClearFootSteps();
    foot_step_list_ = FootStep::GetFwdWalkFootStep(
        n_steps_, nominal_forward_step_, nominal_footwidth_, first_swing_leg_,
        init_mid_foot);
    swing_leg_ =
        first_swing_leg_; // for getter function(GetSwingLeg()) in state machine
    //_AlternateLeg();
    break;
  case dcm_walking_primitive::kBwdWalk:
    _ResetIndexAndClearFootSteps();
    foot_step_list_ = FootStep::GetFwdWalkFootStep(
        n_steps_, nominal_backward_step_, nominal_footwidth_, first_swing_leg_,
        init_mid_foot);
    swing_leg_ =
        first_swing_leg_; // for getter function(GetSwingLeg()) in state machine
    //_AlternateLeg();
    break;
  case dcm_walking_primitive::kInPlaceWalk:
    _ResetIndexAndClearFootSteps();
    foot_step_list_ = FootStep::GetInPlaceWalkFootStep(
        n_steps_, nominal_footwidth_, first_swing_leg_, init_mid_foot);
    swing_leg_ =
        first_swing_leg_; // for getter function(GetSwingLeg()) in state machine
    _AlternateLeg();
    break;

  case dcm_walking_primitive::kLeftTurn:
    _ResetIndexAndClearFootSteps();
    foot_step_list_ = FootStep::GetTurningFootStep(
        n_steps_, nominal_turn_radians_, nominal_footwidth_, init_mid_foot);
    first_swing_leg_ = end_effector::LFoot;
    swing_leg_ =
        first_swing_leg_; // for getter function(GetSwingLeg()) in state machine
    break;

  case dcm_walking_primitive::kRightTurn:
    _ResetIndexAndClearFootSteps();
    foot_step_list_ = FootStep::GetTurningFootStep(
        n_steps_, -nominal_turn_radians_, nominal_footwidth_, init_mid_foot);
    first_swing_leg_ = end_effector::RFoot;
    swing_leg_ =
        first_swing_leg_; // for getter function(GetSwingLeg()) in state machine
    break;

  case dcm_walking_primitive::kLeftStrafe:
    _ResetIndexAndClearFootSteps();
    foot_step_list_ = FootStep::GetStrafeFootStep(
        n_steps_, nominal_strafe_distance_, nominal_footwidth_, init_mid_foot);
    first_swing_leg_ = end_effector::LFoot;
    swing_leg_ =
        first_swing_leg_; // for getter function(GetSwingLeg()) in state machine
    break;

  case dcm_walking_primitive::kRightStrafe:
    _ResetIndexAndClearFootSteps();
    foot_step_list_ = FootStep::GetStrafeFootStep(
        n_steps_, -nominal_strafe_distance_, nominal_footwidth_, init_mid_foot);
    first_swing_leg_ = end_effector::RFoot;
    swing_leg_ =
        first_swing_leg_; // for getter function(GetSwingLeg()) in state machine
    break;

  default:
    std::cerr << "[DCMTrajectoryManager] ERROR. Walking Primitives are not set"
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

bool DCMTrajectoryManager::Initialize(const double t_walk_start,
                                      const int transfer_type,
                                      const Eigen::Quaterniond &init_torso_quat,
                                      const Eigen::Vector3d &init_dcm_pos,
                                      const Eigen::Vector3d &init_dcm_vel) {
  int max_foot_steps_preview(40);
  _UpdateFootStepsPreviewList(max_foot_steps_preview);
  if (foot_step_preview_list_.size() == 0) {
    std::cerr << "[DCMTrajectoryManager] ERROR. Footstep preview list is empty"
              << std::endl;
    return false;
  }

  //---------------------------------------------------------
  // dcm setup
  //---------------------------------------------------------
  dcm_planner_->SetRobotMass(robot_->GetTotalMass());
  dcm_planner_->SetComHeight(init_dcm_pos[2]);
  dcm_planner_->SetInitialTime(t_walk_start);
  dcm_planner_->SetInitialPelvisOri(init_torso_quat);
  // TODO: torso ang vel setting for replanning

  // set transfer time
  double t_transfer_init = dcm_planner_->GetTransferTime();
  double t_transfer_mid = (dcm_planner_->GetAlpha() - 1.0) *
                          dcm_planner_->GetContactTransitionTime();
  if (transfer_type == dcm_transfer_type::kInitial)
    dcm_planner_->SetTransferTime(t_transfer_init);
  else if (transfer_type == dcm_transfer_type::kMidStep)
    dcm_planner_->SetTransferTime(t_transfer_mid);

  dcm_planner_->InitializeFootStepsVrp(foot_step_preview_list_, init_left_foot_,
                                       init_right_foot_, init_dcm_pos,
                                       init_dcm_vel);
  return true;
}

void DCMTrajectoryManager::UpdateDesired(const double current_time) {
  // get reference traj for dcm & com & torso ori

  // Eigen::Vector3d des_dcm_pos = dcm_planner_->GetRefDCM(current_time);
  // Eigen::Vector3d des_dcm_vel = dcm_planner_->GetRefDCMVel(current_time);
  Eigen::Vector3d des_com_pos = dcm_planner_->GetRefCoMPos(current_time);
  Eigen::Vector3d des_com_vel = dcm_planner_->GetRefCoMVel(current_time);
  Eigen::Vector3d des_com_acc = dcm_planner_->GetRefCoMAcc(current_time);

  Eigen::Quaterniond des_ori_quat = Eigen::Quaterniond::Identity();
  Eigen::Vector3d des_ang_vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d des_ang_acc = Eigen::Vector3d::Zero();

  dcm_planner_->GetRefOriAngVelAngAcc(current_time, des_ori_quat, des_ang_vel,
                                      des_ang_acc);
  Eigen::VectorXd des_ori_vec(4);
  des_ori_vec << des_ori_quat.coeffs();

  com_xy_task_->UpdateDesired(des_com_pos.head<2>(), des_com_vel.head<2>(),
                              des_com_acc.head<2>());
  if (!b_use_base_height_)
    com_z_task_->UpdateDesired(des_com_pos.tail<1>(), des_com_vel.tail<1>(),
                               des_com_acc.tail<1>());
  torso_ori_task_->UpdateDesired(des_ori_vec, des_ang_vel, des_ang_acc);
}

void DCMTrajectoryManager::InitializeParameters(const YAML::Node &node) {
  // set dcm params
  dcm_planner_->SetParams(node);

  // set walking primitives parameters (e.g., foot pose)
  util::ReadParameter(node, "nominal_footwidth", nominal_footwidth_);
  util::ReadParameter(node, "nominal_forward_step", nominal_forward_step_);
  util::ReadParameter(node, "nominal_backward_step", nominal_backward_step_);
  util::ReadParameter(node, "nominal_turn_radians", nominal_turn_radians_);
  util::ReadParameter(node, "nominal_strafe_distance",
                      nominal_strafe_distance_);
  util::ReadParameter(node, "n_steps", n_steps_);
  util::ReadParameter(node, "first_swing_leg", first_swing_leg_);
}

int *DCMTrajectoryManager::GetNumSteps() { return &n_steps_; }

void DCMTrajectoryManager::_UpdateFootStepsPreviewList(
    const int max_foot_steps_preview) {
  foot_step_preview_list_.clear();
  for (int preview_idx(0); preview_idx < max_foot_steps_preview;
       ++preview_idx) {
    if (preview_idx + current_foot_step_idx_ < foot_step_list_.size())
      foot_step_preview_list_.push_back(
          foot_step_list_[preview_idx + current_foot_step_idx_]);
    else
      break;
  }
}

void DCMTrajectoryManager::_ResetIndexAndClearFootSteps() {
  current_foot_step_idx_ = 0;
  foot_step_list_.clear();
}

void DCMTrajectoryManager::_AlternateLeg() {
  first_swing_leg_ = first_swing_leg_ == end_effector::LFoot
                         ? end_effector::RFoot
                         : end_effector::LFoot;
}

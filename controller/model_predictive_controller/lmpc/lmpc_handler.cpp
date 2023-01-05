#include "controller/model_predictive_controller/lmpc/lmpc_handler.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"

LMPCHandler::LMPCHandler(DCMPlanner *dcm_planner, PinocchioRobotSystem *robot,
                         Task *com_task, Task *torso_ori_task,
                         ForceTask *lfoot_rf_task, ForceTask *rfoot_rf_task,
                         const int lfoot_idx, const int rfoot_idx)
    : MPCHandler(robot), dcm_planner_(dcm_planner), com_task_(com_task),
      torso_ori_task_(torso_ori_task), lfoot_rf_task_(lfoot_rf_task),
      rfoot_rf_task_(rfoot_rf_task), lfoot_idx_(lfoot_idx),
      rfoot_idx_(rfoot_idx), b_first_visit_(true), walking_primitive_(-1),
      first_swing_leg_(end_effector::LFoot), current_foot_step_idx_(0) {
  util::PrettyConstructor(2, "LMPCHandler");
  foot_step_list_.clear();
  foot_step_preview_list_.clear();
}

void LMPCHandler::GenerateFootSteps() {
  //---------------------------------------------------------
  // foot step setup
  //---------------------------------------------------------
  // update current stance feet
  Eigen::Isometry3d lfoot_iso = robot_->GetLinkIsometry(lfoot_idx_);
  init_left_foot_.SetPosOriSide(lfoot_iso.translation(),
                                Eigen::Quaterniond(lfoot_iso.linear()),
                                end_effector::LFoot);

  Eigen::Isometry3d rfoot_iso = robot_->GetLinkIsometry(rfoot_idx_);
  init_right_foot_.SetPosOriSide(rfoot_iso.translation(),
                                 Eigen::Quaterniond(rfoot_iso.linear()),
                                 end_effector::RFoot);

  FootStep init_mid_foot;
  FootStep::ComputeMidFoot(init_left_foot_, init_right_foot_, init_mid_foot);

  // generate foot step list depening on walking primitives
  switch (walking_primitive_) {
  case lmpc_walking_primitive::kFwdWalk:
    _ResetIndexAndClearFootSteps();
    foot_step_list_ = FootStep::GetFwdWalkFootStep(
        n_steps_, nominal_forward_step_, nominal_footwidth_, first_swing_leg_,
        init_mid_foot);
    _AlternateLeg();
    break;
  case lmpc_walking_primitive::kBwdWalk:
    _ResetIndexAndClearFootSteps();
    foot_step_list_ = FootStep::GetFwdWalkFootStep(
        n_steps_, nominal_backward_step_, nominal_footwidth_, first_swing_leg_,
        init_mid_foot);
    _AlternateLeg();
    break;
  case lmpc_walking_primitive::kInPlaceWalk:
    _ResetIndexAndClearFootSteps();
    foot_step_list_ = FootStep::GetInPlaceWalkFootStep(
        n_steps_, nominal_footwidth_, first_swing_leg_, init_mid_foot);
    _AlternateLeg();
    break;

  case lmpc_walking_primitive::kLeftTurn:
    _ResetIndexAndClearFootSteps();
    foot_step_list_ = FootStep::GetTurningFootStep(
        n_steps_, nominal_turn_radians_, nominal_footwidth_, init_mid_foot);
    break;

  case lmpc_walking_primitive::kRightTurn:
    _ResetIndexAndClearFootSteps();
    foot_step_list_ = FootStep::GetTurningFootStep(
        n_steps_, -nominal_turn_radians_, nominal_footwidth_, init_mid_foot);
    break;

  case lmpc_walking_primitive::kLeftStrafe:
    _ResetIndexAndClearFootSteps();
    foot_step_list_ = FootStep::GetStrafeFootStep(
        n_steps_, nominal_strafe_distance_, nominal_footwidth_, init_mid_foot);
    break;

  case lmpc_walking_primitive::kRightStrafe:
    _ResetIndexAndClearFootSteps();
    foot_step_list_ = FootStep::GetStrafeFootStep(
        n_steps_, -nominal_strafe_distance_, nominal_footwidth_, init_mid_foot);
    break;

  default:
    std::cerr << "[LMPCHandler] ERROR. Walking Primitives are not set"
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

bool LMPCHandler::UpdateReferenceTrajectory(
    const double t_walk_start, const int transfer_type,
    const Eigen::Quaterniond &init_torso_quat,
    const Eigen::Vector3d &init_dcm_pos, const Eigen::Vector3d &init_dcm_vel) {

  int max_foot_steps_preview(40);
  _UpdateFootStepsPreviewList(max_foot_steps_preview);
  if (foot_step_preview_list_.size() == 0) {
    std::cerr << "[LMPCHandler] ERROR. Footstep preview list is empty"
              << std::endl;
    return false;
  }

  //---------------------------------------------------------
  // dcm setup
  //---------------------------------------------------------
  dcm_planner_->SetRobotMass(robot_->GetTotalMass());
  dcm_planner_->SetInitialTime(t_walk_start);
  dcm_planner_->SetInitialPelvisOri(init_torso_quat);
  // TODO: dcm_planner_->SetInitialPelvisAngVel(init_tosro_ang_vel);

  // set transfer time
  double t_transfer_init = dcm_planner_->GetTransferTime();
  double t_transfer_mid = (dcm_planner_->GetAlpha() - 1.0) *
                          dcm_planner_->GetContactTransitionTime();
  if (transfer_type == dcm_transfer_type::kInitial)
    dcm_planner_->SetTransferTime(t_transfer_init);
  else if (transfer_type == dcm_transfer_type::kMidStep)
    dcm_planner_->SetTransferTime(t_transfer_mid);

  // compute dcm trajectory based on the foot step list
  dcm_planner_->InitializeFootStepsVrp(foot_step_preview_list_, init_left_foot_,
                                       init_right_foot_, init_dcm_pos,
                                       init_dcm_vel);
  return true;
}

void LMPCHandler::UpdateDesired(const double current_time) {}

void LMPCHandler::InitializeParameters(const YAML::Node &node) {
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

void LMPCHandler::_UpdateFootStepsPreviewList(
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

void LMPCHandler::_ResetIndexAndClearFootSteps() {
  current_foot_step_idx_ = 0;
  foot_step_list_.clear();
}

void LMPCHandler::_AlternateLeg() {
  first_swing_leg_ = first_swing_leg_ == end_effector::LFoot
                         ? end_effector::RFoot
                         : end_effector::LFoot;
}

void LMPCHandler::_GetMPCInputData() {}

void LMPCHandler::_SendData() {}

void LMPCHandler::_GetMPCOutputData() {}

#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"

DCMTrajectoryManager::DCMTrajectoryManager(DCMPlanner *dcm_planner,
                                           Task *com_task, Task *torso_ori_task,
                                           PinocchioRobotSystem *robot,
                                           const int lfoot_idx,
                                           const int rfoot_idx)
    : dcm_planner_(dcm_planner), com_task_(com_task),
      torso_ori_task_(torso_ori_task), robot_(robot), lfoot_idx_(lfoot_idx),
      rfoot_idx_(rfoot_idx), current_foot_step_idx_(0),
      first_swing_leg_(end_effector::LFoot) {
  util::PrettyConstructor(2, "DCMTrajectoryManager");
}

bool DCMTrajectoryManager::Initialize(const double t_walk_start,
                                      const int transfer_type,
                                      const Eigen::Quaterniond &init_torso_quat,
                                      const Eigen::Vector3d &init_dcm_pos,
                                      const Eigen::Vector3d &init_dcm_vel) {
  if (foot_step_list_.size() == 0)
    return false;

  //---------------------------------------------------------
  // foot step setup
  //---------------------------------------------------------
  // update current stance feet
  _UpdateStartingStanceFeet();
  init_left_foot_ = current_left_foot_;
  init_right_foot_ = current_right_foot_;
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
  dcm_planner_->SetInitialTime(t_walk_start);
  dcm_planner_->SetInitialPelvisOri(init_torso_quat);

  // set transfer time
  double t_transfer_init = dcm_planner_->GetTransferTime();
  double t_transfer_mid = (dcm_planner_->GetAlpha() - 1.0) *
                          dcm_planner_->GetContactTransitionTime();
  if (transfer_type == dcm_transfer_type::kInitial)
    dcm_planner_->SetTransferTime(t_transfer_init);
  else if (transfer_type == dcm_transfer_type::kMidStep)
    dcm_planner_->SetTransferTime(
        t_transfer_mid); // TODO: not sure why we need it

  dcm_planner_->InitializeFootStepsVrp(foot_step_preview_list_, init_left_foot_,
                                       init_right_foot_, init_dcm_pos,
                                       init_dcm_vel);
  return true;
}

void DCMTrajectoryManager::UpdateDesired(const int current_time) {
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

  com_task_->UpdateDesired(des_com_pos, des_com_vel, des_com_acc);
  torso_ori_task_->UpdateDesired(des_ori_vec, des_ang_vel, des_ang_acc);
}

double DCMTrajectoryManager::GetInitialContactTransferTime() const {
  return dcm_planner_->GetTransferTime() +
         dcm_planner_->GetContactTransitionTime() +
         (1 - dcm_planner_->GetAlpha()) *
             dcm_planner_->GetContactTransitionTime();
}

double DCMTrajectoryManager::GetMidStepContactTransferTime() const {
  return dcm_planner_->GetContactTransitionTime();
}

double DCMTrajectoryManager::GetFinalContactTransferTime() const {
  return dcm_planner_->GetContactTransitionTime() +
         dcm_planner_->GetSettleTime();
}
double DCMTrajectoryManager::GetSwingTime() const {
  return dcm_planner_->GetSwingTime();
}
double DCMTrajectoryManager::GetNormalForceRampUpTime() const {
  return dcm_planner_->GetAlpha() * dcm_planner_->GetContactTransitionTime();
}
double DCMTrajectoryManager::GetNormalForceRampDownTime() const {
  return (1. - dcm_planner_->GetAlpha()) *
         dcm_planner_->GetContactTransitionTime();
}

void DCMTrajectoryManager::_UpdateStartingStanceFeet() {
  Eigen::Isometry3d lfoot_iso = robot_->GetLinkIsometry(lfoot_idx_);
  current_left_foot_.SetPosOriSide(lfoot_iso.translation(),
                                   Eigen::Quaterniond(lfoot_iso.linear()),
                                   end_effector::LFoot);

  Eigen::Isometry3d rfoot_iso = robot_->GetLinkIsometry(rfoot_idx_);
  current_right_foot_.SetPosOriSide(rfoot_iso.translation(),
                                    Eigen::Quaterniond(rfoot_iso.linear()),
                                    end_effector::RFoot);

  current_mid_foot_.ComputeMidFoot(current_left_foot_, current_right_foot_,
                                   current_mid_foot_);
}

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

void DCMTrajectoryManager::_PopulateWalkForward(const int n_steps,
                                                const double forward_distance,
                                                const int first_swing_leg) {
  _UpdateStartingStanceFeet();

  FootStep new_foot_step;
  FootStep current_mid_foot = current_mid_foot_;

  // left foot swing first
  int swing_leg_side = first_swing_leg;
  for (int i(0); i < n_steps; ++i) {
    if (swing_leg_side == end_effector::LFoot) {
      Eigen::Vector3d local_offset(static_cast<double>(i + 1) *
                                       forward_distance,
                                   nominal_footwidth_ / 2., 0.);
      new_foot_step.SetPosOriSide(
          current_mid_foot.GetPos() +
              current_mid_foot.GetRotMat() * local_offset,
          current_mid_foot.GetOrientation(), end_effector::LFoot);
      swing_leg_side = end_effector::RFoot;
    } else {
      Eigen::Vector3d local_offset(static_cast<double>(i + 1) *
                                       forward_distance,
                                   -nominal_footwidth_ / 2., 0.);
      new_foot_step.SetPosOriSide(
          current_mid_foot.GetPos() +
              current_mid_foot.GetRotMat() * local_offset,
          current_mid_foot.GetOrientation(), end_effector::RFoot);
      swing_leg_side = end_effector::LFoot;
    }
    foot_step_list_.push_back(new_foot_step);
  }

  // add additional step forward to square the feet
  if (swing_leg_side == end_effector::LFoot) {
    Eigen::Vector3d local_offset(static_cast<double>(n_steps) *
                                     forward_distance,
                                 nominal_footwidth_ / 2., 0.);
    new_foot_step.SetPosOriSide(
        current_mid_foot.GetPos() + current_mid_foot.GetRotMat() * local_offset,
        current_mid_foot.GetOrientation(), end_effector::LFoot);
  } else {
    Eigen::Vector3d local_offset(static_cast<double>(n_steps) *
                                     forward_distance,
                                 -nominal_footwidth_ / 2., 0.);
    new_foot_step.SetPosOriSide(
        current_mid_foot.GetPos() + current_mid_foot.GetRotMat() * local_offset,
        current_mid_foot.GetOrientation(), end_effector::RFoot);
  }
  foot_step_list_.push_back(new_foot_step);
}

void DCMTrajectoryManager::_PopulateStepsInPlace(const int n_steps,
                                                 const int first_swing_leg) {
  _UpdateStartingStanceFeet();

  FootStep new_foot_step;
  FootStep current_mid_foot = current_mid_foot_;

  int swing_leg_side = first_swing_leg;
  for (int i(0); i < n_steps; ++i) {
    if (swing_leg_side == end_effector::LFoot) {
      Eigen::Vector3d local_offset(0., nominal_footwidth_ / 2., 0.);
      new_foot_step.SetPosOriSide(
          current_mid_foot.GetPos() +
              current_mid_foot.GetRotMat() * local_offset,
          current_mid_foot.GetOrientation(), end_effector::LFoot);
      swing_leg_side = end_effector::RFoot;
    } else {
      Eigen::Vector3d local_offset(0., -nominal_footwidth_ / 2., 0.);
      new_foot_step.SetPosOriSide(
          current_mid_foot.GetPos() +
              current_mid_foot.GetRotMat() * local_offset,
          current_mid_foot.GetOrientation(), end_effector::RFoot);
      swing_leg_side = end_effector::LFoot;
    }
    foot_step_list_.push_back(new_foot_step);
  }
}

// always even n_steps to make both feet parallel
void DCMTrajectoryManager::_PopulateRotateTurn(
    const int n_steps, const double turn_radians_per_step) {
  _UpdateStartingStanceFeet();

  FootStep new_right_foot, new_left_foot;
  FootStep rotated_mid_foot = current_mid_foot_;

  Eigen::Quaterniond quat_increment_local(
      Eigen::AngleAxisd(turn_radians_per_step, Eigen::Vector3d::UnitZ()));
  Eigen::Vector3d local_offset(0., nominal_footwidth_ / 2., 0);
  for (int i(0); i < n_steps; ++i) {
    rotated_mid_foot.SetOri(quat_increment_local *
                            rotated_mid_foot.GetOrientation());

    new_left_foot.SetPosOriSide(
        rotated_mid_foot.GetPos() + rotated_mid_foot.GetRotMat() * local_offset,
        rotated_mid_foot.GetOrientation(), end_effector::LFoot);
    new_right_foot.SetPosOriSide(
        rotated_mid_foot.GetPos() +
            rotated_mid_foot.GetRotMat() * -local_offset,
        rotated_mid_foot.GetOrientation(), end_effector::RFoot);
    if (turn_radians_per_step > 0) {
      foot_step_list_.push_back(new_left_foot);
      foot_step_list_.push_back(new_right_foot);
    } else {
      foot_step_list_.push_back(new_right_foot);
      foot_step_list_.push_back(new_left_foot);
    }
  }
}

void DCMTrajectoryManager::_PopulateStrafe(const int n_steps,
                                           const double strafe_distance) {
  _UpdateStartingStanceFeet();

  FootStep new_right_foot, new_left_foot;
  FootStep strafed_mid_foot = current_mid_foot_;

  Eigen::Vector3d strafe_vec(0., strafe_distance, 0.);
  Eigen::Vector3d local_offset(0., nominal_footwidth_ / 2., 0.);
  for (int i(0); i < n_steps; ++i) {
    strafed_mid_foot.SetPos(strafed_mid_foot.GetPos() +
                            strafed_mid_foot.GetRotMat() * strafe_vec);

    new_left_foot.SetPosOriSide(
        strafed_mid_foot.GetPos() + strafed_mid_foot.GetRotMat() * local_offset,
        strafed_mid_foot.GetOrientation(), end_effector::LFoot);
    new_right_foot.SetPosOriSide(
        strafed_mid_foot.GetPos() +
            strafed_mid_foot.GetRotMat() * -local_offset,
        strafed_mid_foot.GetOrientation(), end_effector::RFoot);

    if (strafe_distance > 0) {
      foot_step_list_.push_back(new_left_foot);
      foot_step_list_.push_back(new_right_foot);
    } else {
      foot_step_list_.push_back(new_right_foot);
      foot_step_list_.push_back(new_left_foot);
    }
  }
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
}

void DCMTrajectoryManager::SaveSolution(const std::string &file_name) {
  try {
    double t_start = dcm_planner_->GetInitialTime();
    double t_end =
        dcm_planner_->GetInitialTime() + dcm_planner_->GetTotalTrajTime();
    double t_step(0.01);
    int n_node = std::floor((t_end - t_start) / t_step);

    YAML::Node cfg;

    // =====================================================================
    // Temporal Parameters
    // =====================================================================
    cfg["temporal_parameters"]["initial_time"] = t_start;
    cfg["temporal_parameters"]["final_time"] = t_end;
    cfg["temporal_parameters"]["time_step"] = t_step;
    cfg["temporal_parameters"]["t_ds"] =
        dcm_planner_->GetContactTransitionTime();
    cfg["temporal_parameters"]["t_ss"] = dcm_planner_->GetSwingTime();
    cfg["temporal_parameters"]["t_transfer"] = dcm_planner_->GetTransferTime();

    // =====================================================================
    // Contact Information
    // =====================================================================
    Eigen::MatrixXd curr_rfoot_pos = Eigen::MatrixXd::Zero(1, 3);
    Eigen::MatrixXd curr_rfoot_quat = Eigen::MatrixXd::Zero(1, 4);
    Eigen::MatrixXd curr_lfoot_pos = Eigen::MatrixXd::Zero(1, 3);
    Eigen::MatrixXd curr_lfoot_quat = Eigen::MatrixXd::Zero(1, 4);
    for (int i = 0; i < 3; ++i) {
      curr_rfoot_pos(0, i) = init_right_foot_.GetPos()[i];
      curr_lfoot_pos(0, i) = init_left_foot_.GetPos()[i];
    }
    curr_rfoot_quat(0, 0) = init_right_foot_.GetOrientation().w();
    curr_rfoot_quat(0, 1) = init_right_foot_.GetOrientation().x();
    curr_rfoot_quat(0, 2) = init_right_foot_.GetOrientation().y();
    curr_rfoot_quat(0, 3) = init_right_foot_.GetOrientation().z();

    curr_lfoot_quat(0, 0) = init_left_foot_.GetOrientation().w();
    curr_lfoot_quat(0, 1) = init_left_foot_.GetOrientation().x();
    curr_lfoot_quat(0, 2) = init_left_foot_.GetOrientation().y();
    curr_lfoot_quat(0, 3) = init_left_foot_.GetOrientation().z();

    int n_rf(0);
    int n_lf(0);
    for (int i(0); i < foot_step_list_.size(); ++i) {
      if (foot_step_list_[i].GetFootSide() == end_effector::LFoot) {
        n_lf += 1;
      } else {
        n_rf += 1;
      }
    }
    Eigen::MatrixXd rfoot_pos = Eigen::MatrixXd::Zero(n_rf, 3);
    Eigen::MatrixXd rfoot_quat = Eigen::MatrixXd::Zero(n_rf, 4);
    Eigen::MatrixXd lfoot_pos = Eigen::MatrixXd::Zero(n_lf, 3);
    Eigen::MatrixXd lfoot_quat = Eigen::MatrixXd::Zero(n_lf, 4);
    int rf_id(0);
    int lf_id(0);
    for (int i(0); i < foot_step_list_.size(); ++i) {
      if (foot_step_list_[i].GetFootSide() == end_effector::RFoot) {
        for (int j(0); j < 3; ++j) {
          rfoot_pos(rf_id, j) = foot_step_list_[i].GetPos()[j];
        }
        rfoot_quat(rf_id, 0) = foot_step_list_[i].GetOrientation().w();
        rfoot_quat(rf_id, 1) = foot_step_list_[i].GetOrientation().x();
        rfoot_quat(rf_id, 2) = foot_step_list_[i].GetOrientation().y();
        rfoot_quat(rf_id, 3) = foot_step_list_[i].GetOrientation().z();
        rf_id += 1;
      } else {
        for (int j(0); j < 3; ++j) {
          lfoot_pos(lf_id, j) = foot_step_list_[i].GetPos()[j];
        }
        lfoot_quat(lf_id, 0) = foot_step_list_[i].GetOrientation().w();
        lfoot_quat(lf_id, 1) = foot_step_list_[i].GetOrientation().x();
        lfoot_quat(lf_id, 2) = foot_step_list_[i].GetOrientation().y();
        lfoot_quat(lf_id, 3) = foot_step_list_[i].GetOrientation().z();
        lf_id += 1;
      }
    }

    cfg["contact"]["curr_right_foot"]["pos"] = curr_rfoot_pos;
    cfg["contact"]["curr_right_foot"]["ori"] = curr_rfoot_quat;
    cfg["contact"]["curr_left_foot"]["pos"] = curr_lfoot_pos;
    cfg["contact"]["curr_left_foot"]["ori"] = curr_lfoot_quat;
    cfg["contact"]["right_foot"]["pos"] = rfoot_pos;
    cfg["contact"]["right_foot"]["ori"] = rfoot_quat;
    cfg["contact"]["left_foot"]["pos"] = lfoot_pos;
    cfg["contact"]["left_foot"]["ori"] = lfoot_quat;

    // =====================================================================
    // Reference Trajectory
    // =====================================================================
    Eigen::MatrixXd dcm_pos_ref = Eigen::MatrixXd::Zero(n_node, 3);
    Eigen::MatrixXd dcm_vel_ref = Eigen::MatrixXd::Zero(n_node, 3);
    Eigen::MatrixXd com_pos_ref = Eigen::MatrixXd::Zero(n_node, 3);
    Eigen::MatrixXd com_vel_ref = Eigen::MatrixXd::Zero(n_node, 3);
    Eigen::MatrixXd vrp_ref = Eigen::MatrixXd::Zero(n_node, 3);
    Eigen::MatrixXd t_traj = Eigen::MatrixXd::Zero(n_node, 1);

    double t(t_start);
    Eigen::Vector3d temp_vec;
    for (int i(0); i < n_node; ++i) {
      t_traj(i, 0) = t;
      temp_vec = dcm_planner_->GetRefDCM(t);
      for (int j(0); j < 3; ++j) {
        dcm_pos_ref(i, j) = temp_vec(j);
      }
      temp_vec = dcm_planner_->GetRefDCMVel(t);
      for (int j(0); j < 3; ++j) {
        dcm_vel_ref(i, j) = temp_vec(j);
      }
      temp_vec = dcm_planner_->GetRefCoMPos(t);
      for (int j(0); j < 3; ++j) {
        com_pos_ref(i, j) = temp_vec(j);
      }
      temp_vec = dcm_planner_->GetRefCoMVel(t);
      for (int j(0); j < 3; ++j) {
        com_vel_ref(i, j) = temp_vec(j);
      }
      temp_vec = dcm_planner_->GetRefVrp(t);
      for (int j(0); j < 3; ++j) {
        vrp_ref(i, j) = temp_vec(j);
      }
      t += t_step;
    }

    cfg["reference"]["dcm_pos"] = dcm_pos_ref;
    cfg["reference"]["dcm_vel"] = dcm_vel_ref;
    cfg["reference"]["com_pos"] = com_pos_ref;
    cfg["reference"]["com_vel"] = com_vel_ref;
    cfg["reference"]["vrp"] = vrp_ref;
    cfg["reference"]["time"] = t_traj;

    std::string full_path = THIS_COM + std::string("experiment_data/") +
                            file_name + std::string(".yaml");
    std::ofstream file_out(full_path);
    file_out << cfg;

  } catch (YAML::ParserException &e) {
    std::cerr << e.what() << std::endl;
  }
}

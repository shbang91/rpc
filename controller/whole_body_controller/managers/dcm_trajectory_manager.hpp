#pragma once
#include "planner/locomotion/dcm_planner/foot_step.hpp"
#include "util/util.hpp"
#include <Eigen/Dense>

namespace dcm_walking_primitive {
constexpr int kFwdWalk = 0;
constexpr int kBwdWalk = 1;
constexpr int kInPlaceWalk = 2;
constexpr int kRightTurn = 3;
constexpr int kLeftTurn = 4;
constexpr int kRightStrafe = 5;
constexpr int kLeftStrafe = 6;
} // namespace dcm_walking_primitive

class DCMPlanner;
class Task;
class PinocchioRobotSystem;

class DCMTrajectoryManager {
public:
  DCMTrajectoryManager(DCMPlanner *dcm_planner, Task *com_task, Task *torso_ori,
                       PinocchioRobotSystem *robot, const int lfoot_idx,
                       const int rfoot_idx);
  virtual ~DCMTrajectoryManager() = default;

  void InitializeParameters(const YAML::Node &node);

  bool Initialize(const double t_walk_start, const int transfer_type,
                  const Eigen::Quaterniond &init_torso_quat,
                  const Eigen::Vector3d &init_dcm_pos,
                  const Eigen::Vector3d &init_dcm_vel);

  void UpdateDesired(const int current_time);

  // =====================================================================
  // footstep generation methods -> InitializeParameters method need to be
  // called before
  // =====================================================================
  void ForwardWalkMode() {
    walking_primitive_ = dcm_walking_primitive::kFwdWalk;
  }
  void BackwardWalkMode() {
    walking_primitive_ = dcm_walking_primitive::kBwdWalk;
  }
  void InplaceWalkMode() {
    walking_primitive_ = dcm_walking_primitive::kInPlaceWalk;
  }
  void LeftTurnWalkMode() {
    walking_primitive_ = dcm_walking_primitive::kLeftTurn;
  }
  void RightTurnWalkMode() {
    walking_primitive_ = dcm_walking_primitive::kRightTurn;
  }
  void LeftStrafeWalkMode() {
    walking_primitive_ = dcm_walking_primitive::kLeftStrafe;
  }
  void RightStrafeWalkMode() {
    walking_primitive_ = dcm_walking_primitive::kRightStrafe;
  }

  // =====================================================================
  // methods to be used in finite state machines
  // =====================================================================
  double GetInitialContactTransferTime() const;
  double GetMidStepContactTransferTime() const;
  double GetFinalContactTransferTime() const;
  double GetSwingTime() const;
  double GetNormalForceRampUpTime() const;
  double GetNormalForceRampDownTime() const;
  void IncrementStepIndex() { ++current_foot_step_idx_; }
  bool NoRemainingSteps() {
    return current_foot_step_idx_ >= foot_step_list_.size() ? true : false;
  }

  // getter
  DCMPlanner *GetDCMPlanner() { return dcm_planner_; }

private:
  DCMPlanner *dcm_planner_;
  Task *com_task_;
  Task *torso_ori_task_;
  PinocchioRobotSystem *robot_;
  int lfoot_idx_;
  int rfoot_idx_;

  bool b_first_visit_;
  std::vector<FootStep> foot_step_list_;
  std::vector<FootStep> foot_step_preview_list_;

  int current_foot_step_idx_;

  int first_swing_leg_;

  // walking primitives
  int walking_primitive_;

  double nominal_footwidth_ = 0.27;
  double nominal_forward_step_ = 0.25;
  double nominal_backward_step_ = -0.25;
  double nominal_turn_radians_ = M_PI / 4.0;
  double nominal_strafe_distance_ = 0.125;
  int n_steps_ = 3;

  // for footstep generation
  void _UpdateFootStepsPreviewList(const int max_foot_steps_preview);
  void _ResetIndexAndClearFootSteps();
  void _AlternateLeg();
};

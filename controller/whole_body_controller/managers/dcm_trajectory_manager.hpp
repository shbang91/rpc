#pragma once
#include "planner/locomotion/dcm_planner/foot_step.hpp"
#include "util/util.hpp"
#include <Eigen/Dense>

namespace dcm_transfer_type {
constexpr int kInitial = 0;
constexpr int kMidStep = 1;
}; // namespace dcm_transfer_type

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

  void SaveSolution(const std::string &file_name);

  // =====================================================================
  // footstep generation methods -> InitializeParameters method need to be
  // called before
  // =====================================================================
  void WalkForward() {
    _ResetIndexAndClearFootSteps();
    _PopulateWalkForward(n_steps_, nominal_forward_step_, first_swing_leg_);
    _AlternateLeg();
  }

  void WalkBackward() {
    _ResetIndexAndClearFootSteps();
    _PopulateWalkForward(n_steps_, nominal_backward_step_, first_swing_leg_);
    _AlternateLeg();
  }

  void WalkInPlace() {
    _ResetIndexAndClearFootSteps();
    _PopulateStepsInPlace(n_steps_, first_swing_leg_);
    _AlternateLeg();
  }

  void TurnRight() {
    _ResetIndexAndClearFootSteps();
    _PopulateRotateTurn(n_steps_, -nominal_turn_radians_);
  }

  void TurnLeft() {
    _ResetIndexAndClearFootSteps();
    _PopulateRotateTurn(n_steps_, nominal_turn_radians_);
  }

  void StrafeRight() {
    _ResetIndexAndClearFootSteps();
    _PopulateStrafe(n_steps_, -nominal_strafe_distance_);
  }

  void StrafeLeft() {
    _ResetIndexAndClearFootSteps();
    _PopulateStrafe(n_steps_, nominal_strafe_distance_);
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

private:
  DCMPlanner *dcm_planner_;
  Task *com_task_;
  Task *torso_ori_task_;
  PinocchioRobotSystem *robot_;
  int lfoot_idx_;
  int rfoot_idx_;

  std::vector<FootStep> foot_step_list_;
  std::vector<FootStep> foot_step_preview_list_;

  FootStep current_left_foot_;
  FootStep current_right_foot_;
  FootStep current_mid_foot_;

  int current_foot_step_idx_;

  int first_swing_leg_;

  // walking primitives
  double nominal_footwidth_ = 0.27;
  double nominal_forward_step_ = 0.25;
  double nominal_backward_step_ = -0.25;
  double nominal_turn_radians_ = M_PI / 4.0;
  double nominal_strafe_distance_ = 0.125;
  int n_steps_ = 3;

  // for saving data
  FootStep init_left_foot_;
  FootStep init_right_foot_;

  void _UpdateStartingStanceFeet();
  void _UpdateFootStepsPreviewList(const int max_foot_steps_preview);

  // for footstep generation
  void _ResetIndexAndClearFootSteps();
  void _AlternateLeg();
  void _PopulateWalkForward(const int n_steps, const double forward_distance,
                            const int first_swing_leg);
  void _PopulateStepsInPlace(const int n_steps, const int first_swing_leg);
  void _PopulateRotateTurn(const int n_steps,
                           const double turn_radians_per_step);
  void _PopulateStrafe(const int n_steps, const double strafe_distance);
};

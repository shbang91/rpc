#pragma once
#include "controller/model_predictive_controller/mpc_handler.hpp"
#include "util/util.hpp"

namespace lmpc_walking_primitive {
constexpr int kFwdWalk = 0;
constexpr int kBwdWalk = 1;
constexpr int kInPlaceWalk = 2;
constexpr int kRightTurn = 3;
constexpr int kLeftTurn = 4;
constexpr int kRightStrafe = 5;
constexpr int kLeftStrafe = 6;
} // namespace lmpc_walking_primitive

class PinocchioRobotSystem;
class Task;
class ForceTask;
class FootStep;
class DCMPlanner;

class LMPCHandler : public MPCHandler {
public:
  // TODO: what about swing foot??
  LMPCHandler(DCMPlanner *dcm_planner, PinocchioRobotSystem *robot,
              Task *com_task, Task *torso_ori_task,
              ForceTask *lfoot_reaction_force_task,
              ForceTask *rfoot_reaction_force_task, const int lfoot_idx,
              const int rfoot_idx);
  virtual ~LMPCHandler() = default;

  bool UpdateReferenceTrajectory(const double t_walk_start,
                                 const int transfer_type,
                                 const Eigen::Quaterniond &init_torso_quat,
                                 const Eigen::Vector3d &init_dcm_pos,
                                 const Eigen::Vector3d &init_dcm_vel);

  void UpdateDesired(const double current_time);

  void InitializeParameters(const YAML::Node &node);

  // =====================================================================
  // footstep generation methods -> InitializeParameters method need to be
  // called before
  // =====================================================================
  void ForwardWalkMode() {
    walking_primitive_ = lmpc_walking_primitive::kFwdWalk;
  }
  void BackwardWalkMode() {
    walking_primitive_ = lmpc_walking_primitive::kBwdWalk;
  }
  void InplaceWalkMode() {
    walking_primitive_ = lmpc_walking_primitive::kInPlaceWalk;
  }
  void LeftTurnWalkMode() {
    walking_primitive_ = lmpc_walking_primitive::kLeftTurn;
  }
  void RightTurnWalkMode() {
    walking_primitive_ = lmpc_walking_primitive::kRightTurn;
  }
  void LeftStrafeWalkMode() {
    walking_primitive_ = lmpc_walking_primitive::kLeftStrafe;
  }
  void RightStrafeWalkMode() {
    walking_primitive_ = lmpc_walking_primitive::kRightStrafe;
  }

  // =====================================================================
  // methods to be used in finite state machines
  // =====================================================================
  void IncrementStepIndex() { ++current_foot_step_idx_; }
  bool NoRemainingSteps() {
    return current_foot_step_idx_ >= foot_step_list_.size() ? true : false;
  }

  // getter
  DCMPlanner *GetDCMPlanner() { return dcm_planner_; }

private:
  void _GetMPCInputData() override;
  void _SendData() override;
  void _GetMPCOutputData() override;

  DCMPlanner *dcm_planner_;
  Task *com_task_;
  Task *torso_ori_task_;
  ForceTask *lfoot_rf_task_;
  ForceTask *rfoot_rf_task_;
  int lfoot_idx_;
  int rfoot_idx_;

  bool b_first_visit_;
  int walking_primitive_;
  std::vector<FootStep> foot_step_list_;
  std::vector<FootStep> foot_step_preview_list_;
  int first_swing_leg_;
  int current_foot_step_idx_;

  double nominal_footwidth_ = 0.27;
  double nominal_forward_step_ = 0.25;
  double nominal_backward_step_ = -0.25;
  double nominal_turn_radians_ = M_PI / 4.0;
  double nominal_strafe_distance_ = 0.125;
  int n_steps_ = 3;

  // foot step related methods
  void _UpdateFootStepsPreviewList(const int max_foot_steps_preview);
  void _ResetIndexAndClearFootSteps();
  void _AlternateLeg();
};

class LMPCInputData : public MPCInputData {
public:
  LMPCInputData();
  virtual ~LMPCInputData() = default;
};

class LMPCOutputData : public MPCOutputData {
public:
  LMPCOutputData();
  virtual ~LMPCOutputData() = default;
};

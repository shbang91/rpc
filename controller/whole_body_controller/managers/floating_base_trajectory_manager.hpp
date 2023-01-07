#pragma once
#include <Eigen/Dense>

#include "configuration.hpp"

class Task;
class PinocchioRobotSystem;
class MinJerkCurveVec;

class FloatingBaseTrajectoryManager {
public:
  FloatingBaseTrajectoryManager(Task *com_xy_task, Task *com_z_task,
                                Task *body_ori_task,
                                PinocchioRobotSystem *robot);
  ~FloatingBaseTrajectoryManager();

  void InitializeFloatingBaseInterpolation(
      const Eigen::Vector3d &init_com_pos, const Eigen::Vector3d &des_com_pos,
      const Eigen::Quaterniond &init_torso_quat,
      const Eigen::Quaterniond &target_torso_quat, const double duration);

  void InitializeSwaying(const Eigen::Vector3d &init_com_pos,
                         const Eigen::Vector3d &amp,
                         const Eigen::Vector3d &freq);

  void UpdateDesired(const double state_machine_time);

private:
  Task *com_xy_task_;
  Task *com_z_task_;
  Task *torso_ori_task_;
  PinocchioRobotSystem *robot_;

  // floating base interpolation
  double duration_;

  Eigen::Vector3d init_com_pos_;
  Eigen::Vector3d target_com_pos_;

  Eigen::Quaterniond init_torso_quat_;
  Eigen::VectorXd exp_err_;

  MinJerkCurveVec *min_jerk_curve_;
  MinJerkCurveVec *min_jerk_time_;

  // com swaying
  Eigen::Vector3d amp_;
  Eigen::Vector3d freq_;
  bool b_swaying_;
};

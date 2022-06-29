#pragma once
#include <Eigen/Dense>

class Task;
class PinocchioRobotSystem;

class FloatingBaseTrajectoryManager {
public:
  FloatingBaseTrajectoryManager(Task *com_task, Task *body_ori_task,
                                PinocchioRobotSystem *robot);
  virtual ~FloatingBaseTrajectoryManager() = default;

  void InitializeFloatingBaseInterpolation(
      const Eigen::Vector3d &init_com_pos, const Eigen::Vector3d &des_com_pos,
      const Eigen::Quaterniond &init_torso_quat,
      const Eigen::Quaterniond &target_torso_quat, const double duration);
  void UpdateDesired(const double state_machine_time);

  void InitializeSwaying(const Eigen::Vector3d &init_com_pos,
                         const Eigen::Vector3d &amp,
                         const Eigen::Vector3d &freq);

private:
  Task *com_task_;
  Task *torso_ori_task_;
  PinocchioRobotSystem *robot_;

  // floating base interpolation
  double duration_;
  Eigen::Vector3d init_com_pos_;
  Eigen::Vector3d target_com_pos_;

  Eigen::Quaterniond init_torso_quat_;
  Eigen::VectorXd exp_err_;

  // com swaying
  Eigen::Vector3d amp_;
  Eigen::Vector3d freq_;
  bool b_swaying_;
};

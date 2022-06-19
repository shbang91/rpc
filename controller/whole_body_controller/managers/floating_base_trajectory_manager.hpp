#pragma once
#include <Eigen/Dense>

class Task;
class PinocchioRobotSystem;

class FloatingBaseTrajectoryManager {
public:
  FloatingBaseTrajectoryManager(Task *com_task, Task *body_ori_task,
                                PinocchioRobotSystem *robot);
  virtual ~FloatingBaseTrajectoryManager() = default;

  void
  InitializeFloatingBaseInterpolation(const Eigen::Vector3d &des_com_pos,
                                      const Eigen::Quaterniond &des_body_quat,
                                      const double &duration,
                                      const bool &b_use_base_height);
  void UpdateDesired(const double &state_machine_time);

private:
  Task *com_task_;
  Task *torso_ori_task_;
  PinocchioRobotSystem *robot_;

  double duration_;
  Eigen::Vector3d init_com_pos_;
  Eigen::Vector3d target_com_pos_;

  Eigen::Quaterniond init_torso_quat_;
  Eigen::VectorXd exp_err_;
};

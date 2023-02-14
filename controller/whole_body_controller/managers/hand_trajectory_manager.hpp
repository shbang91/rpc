#pragma once
#include <Eigen/Dense>

class Task;
class PinocchioRobotSystem;
class HermiteQuaternionCurve;
class HermiteCurveVec;

class HandTrajectoryManager {
public:
  HandTrajectoryManager(Task *pos_task, Task *ori_task,
                               PinocchioRobotSystem *robot);
  ~HandTrajectoryManager();

  void UseCurrent();

  void InitializeHandTrajectory( const Eigen::Isometry3d &target_pose,
                                 const double start_time,
                                 const double duration);
  void UpdateHandPose(const double current_time);
  void UpdateDesired(const Eigen::Isometry3d &target_pose);

private:
  Task *pos_task_;
  Task *ori_task_;
  PinocchioRobotSystem *robot_;

  HermiteCurveVec *pos_curve_;
  HermiteQuaternionCurve *ori_curve_;

  Eigen::VectorXd target_pos_;
  Eigen::Quaterniond target_ori_;

  int initialized_ = 0;
  double duration_ = 0.;
  double start_time_ = 0.;
};

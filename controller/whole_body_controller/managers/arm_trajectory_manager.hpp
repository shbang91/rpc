#pragma once
#include <Eigen/Dense>

class Task;
class PinocchioRobotSystem;
class HermiteCurveVec;
class HermiteQuaternionCurve;

class ArmTrajectoryManager {
public:
  ArmTrajectoryManager(Task *pos_task, Task *ori_task,
                               PinocchioRobotSystem *robot);
  ~ArmTrajectoryManager();

  void UseCurrent();

  void UseNominal(const Eigen::Isometry3d &nominal_iso);

  void InitializeTrajectory(const Eigen::Isometry3d &ini_pose,
                                 const Eigen::Isometry3d &fin_pose,
                                 const double duration);
  void UpdateDesired(const double current_time);

private:
  Task *pos_task_;
  Task *ori_task_;
  PinocchioRobotSystem *robot_;

  HermiteCurveVec *pos_curve_;
  HermiteQuaternionCurve *ori_curve_;

  double duration_ = 0.;
};

#pragma once
#include <Eigen/Dense>

class Task;
class PinocchioRobotSystem;
class UpperBodyTrajetoryManager {
public:
  UpperBodyTrajetoryManager(Task *upper_body_task, PinocchioRobotSystem *robot);
  virtual ~UpperBodyTrajetoryManager() = default;

  void UseNominalUpperBodyJointPos(const Eigen::VectorXd &nominal_jpos);

private:
  Task *upper_body_task_;
  PinocchioRobotSystem *robot_;
};

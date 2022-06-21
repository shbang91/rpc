#pragma once
#include <vector>

class Task;
class Contact;
class InternalConstraint;
class ForceTask;
class PinocchioRobotSystem;

class TCIContainer {
public:
  TCIContainer(PinocchioRobotSystem *robot) { robot_ = robot; };
  virtual ~TCIContainer() = default;

  std::vector<Task *> task_container_;
  std::vector<Contact *> contact_container_;
  std::vector<InternalConstraint *> internal_constraint_container_;
  std::vector<ForceTask *> force_container_;

protected:
  PinocchioRobotSystem *robot_;
};

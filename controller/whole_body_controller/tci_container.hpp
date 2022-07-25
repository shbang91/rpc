#pragma once
#include <vector>
#include <unordered_map>

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
  std::unordered_map<std::string, Task *> task_map_;
  std::vector<Contact *> contact_container_;
  std::unordered_map<std::string, Contact *> contact_map_;
  std::vector<InternalConstraint *> internal_constraint_container_;
  std::vector<ForceTask *> force_task_container_;
  std::unordered_map<std::string, ForceTask *> force_task_map_;

protected:
  PinocchioRobotSystem *robot_;
};

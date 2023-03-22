#pragma once
#include <map>
#include <string>
#include <unordered_map>
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

  std::unordered_map<std::string, Task *> task_map_;
  std::vector<Task *> task_vector_;
  std::map<std::string, Contact *> contact_map_;
  std::vector<Contact *> contact_vector_;
  std::vector<InternalConstraint *> internal_constraint_vector_;
  std::unordered_map<std::string, ForceTask *> force_task_map_;
  std::vector<ForceTask *> force_task_vector_;

protected:
  PinocchioRobotSystem *robot_;
};

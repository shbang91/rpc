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

  // IHWBC
  std::unordered_map<std::string, Task *> task_map_;
  std::map<std::string, Contact *> contact_map_;
  std::unordered_map<std::string, InternalConstraint *>
      internal_constraint_map_;
  std::map<std::string, ForceTask *> force_task_map_;
  std::unordered_map<std::string, double>
      task_unweighted_cost_map_; // unweighted task costs
  std::unordered_map<std::string, double>
      task_weighted_cost_map_; // unweighted task costs

  // WBIC
  std::vector<Task *> task_vector_;
  std::vector<Contact *> contact_vector_;
  std::vector<InternalConstraint *> internal_constraint_vector_;
  std::vector<ForceTask *> force_task_vector_;

protected:
  PinocchioRobotSystem *robot_;
};

#pragma once
#include <vector>

#include "controller/whole_body_controller/contact.hpp"
#include "controller/whole_body_controller/internal_constraint.hpp"
#include "controller/whole_body_controller/task.hpp"

class PinocchioRobotSystem;

class TCIContainer {
public:
  TCIContainer(PinocchioRobotSystem *robot) { robot_ = robot; };
  virtual ~TCIContainer() = default;

  std::vector<Task *> task_container_;
  std::vector<Contact *> contact_container_;
  std::vector<InternalConstraint *> internal_constraint_container_;

protected:
  PinocchioRobotSystem *robot_;
};

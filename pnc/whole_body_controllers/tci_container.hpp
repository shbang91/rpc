#pragma once
#include <vector>

#include "pnc/whole_body_controllers/contact.hpp"
#include "pnc/whole_body_controllers/internal_constraint.hpp"
#include "pnc/whole_body_controllers/task.hpp"

class RobotSystem;

class TCIContainer {
public:
  TCIContainer(RobotSystem *_robot) { robot_ = _robot; };
  virtual ~TCIContainer() = default;

  std::vector<Task *> task_container_;
  std::vector<Contact *> contact_container_;
  std::vector<InternalConstraint *> internal_constraint_container_;

protected:
  RobotSystem *robot_;
};

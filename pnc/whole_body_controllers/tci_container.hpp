#pragma once
#include <vector>

#include "pnc/robot_system/robot_system.hpp"
#include "pnc/whole_body_controllers/contact.hpp"
#include "pnc/whole_body_controllers/internal_constraint.hpp"
#include "pnc/whole_body_controllers/task.hpp"

class TCIContainer {
public:
  TCIContainer(RobotSystem *_robot) { robot_ = _robot; };
  virtual ~TCIContainer();

  std::vector<Task *> task_container_;
  std::vector<Contact *> contact_container_;
  std::vector<InternalConstraint *> internal_constraint_container_;

protected:
  RobotSystem *robot_;
}

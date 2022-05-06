#pragma once

#include <map>

#include "pnc/state_machine.hpp"

class RobotSystem;
class ControlArchitecture {
public:
  ControlArchitecture(RobotSystem *_robot) { robot_ = _robot; };

  virtual ~ControlArchitecture() = default;

  virtual void GetCommand(void *_command) = 0;

  std::map<StateId, StateMachine *> state_machines_container_;

  int state_;
  int prev_state_;

protected:
  RobotSystem *robot_;

  bool b_state_first_visit_;
};

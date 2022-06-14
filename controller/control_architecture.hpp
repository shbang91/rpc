#pragma once
#include "controller/state_machine.hpp"
#include <unordered_map>

class PinocchioRobotSystem;

class ControlArchitecture {
public:
  ControlArchitecture(PinocchioRobotSystem *robot) { robot_ = robot; };
  virtual ~ControlArchitecture() = default;

  virtual void GetCommand(void *command) = 0;

  // getter
  int GetStateId() const { return this->state_; }
  int GetPrevStateId() const { return this->prev_state_; }

protected:
  PinocchioRobotSystem *robot_;
  std::unordered_map<StateId, StateMachine *> state_machine_container_;
  int state_;
  int prev_state_;

  bool b_state_first_visit_ = true;
};

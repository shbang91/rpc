#pragma once
#include "controller/state_machine.hpp"
#include <unordered_map>

class PinocchioRobotSystem;

class ControlArchitecture {
public:
  ControlArchitecture(PinocchioRobotSystem *robot)
      : robot_(robot), b_state_first_visit_(true){};
  virtual ~ControlArchitecture() = default;

  virtual void GetCommand(void *command) = 0;

  // getter
  StateId state() const { return this->state_; }
  StateId prev_state() const { return this->prev_state_; }
  std::unordered_map<StateId, StateMachine *> state_machine_container() const {
    return this->state_machine_container_;
  }
  virtual void Reset() = 0;

protected:
  PinocchioRobotSystem *robot_;
  bool b_state_first_visit_;

  std::unordered_map<StateId, StateMachine *> state_machine_container_;
  StateId state_;
  StateId prev_state_;
  YAML::Node cfg_;
  virtual void _InitializeParameters() = 0;
};

#pragma once
#include "controller/state_machine.hpp"
#include <unordered_map>

class PinocchioRobotSystem;

class ControlArchitecture {
public:
  ControlArchitecture(PinocchioRobotSystem *robot)
      : robot_(robot), b_loco_state_first_visit_(true),
        b_manip_state_first_visit_(true){};
  virtual ~ControlArchitecture() = default;

  virtual void GetCommand(void *command) = 0;

  // getter
  StateId locostate() const { return this->loco_state_; }
  StateId prev_locostate() const { return this->prev_loco_state_; }
  std::unordered_map<StateId, StateMachine *>
  locomotion_state_machine_container() const {
    return this->locomotion_state_machine_container_;
  }
  std::unordered_map<StateId, StateMachine *>
  manipulation_state_machine_container() const {
    return this->manipulation_state_machine_container_;
  }

protected:
  PinocchioRobotSystem *robot_;
  bool b_loco_state_first_visit_;
  bool b_manip_state_first_visit_;

  std::unordered_map<StateId, StateMachine *>
      locomotion_state_machine_container_;
  std::unordered_map<StateId, StateMachine *>
      manipulation_state_machine_container_;
  StateId loco_state_;
  StateId prev_loco_state_;
  StateId manip_state_;
  StateId prev_manip_state_;
};

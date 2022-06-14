#pragma once

typedef int StateId;
class PinocchioRobotSystem;

class StateMachine {
public:
  StateMachine(StateId state_id, PinocchioRobotSystem *robot)
      : state_id_(state_id), state_machine_time_(0.) {
    robot_ = robot;
  }
  virtual ~StateMachine() = default;

  virtual void FirstVisit() = 0;
  virtual void OneStep() = 0;
  virtual void LastVisit() = 0;
  virtual bool EndOfState() = 0;

  virtual StateId GetNextState() = 0;

  StateId GetStateId() { return this->state_id_; }

protected:
  StateId state_id_;
  PinocchioRobotSystem *robot_;
  double state_machine_time_;
};

#pragma once

typedef int StateId;

class RobotSystem;

class StateMachine {
public:
  StateMachine(StateId _state_id, RobotSystem *_robot) {
    state_id_ = _state_id;
    robot_ = _robot;
    state_machine_time_ = 0.;
  };

  virtual ~StateMachine() = default;

  virtual void FirstVisit() = 0;

  virtual void OneStep() = 0;

  virtual void LastVisit() = 0;

  virtual bool EndOfState() = 0;

  virtual StateId GetNextState() = 0;

  StateId GetStateId() { return state_id_; };

protected:
  StateId state_id_;
  double state_machine_time_;
  RobotSystem *robot_;
};

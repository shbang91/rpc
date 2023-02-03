#pragma once
#include "util/util.hpp"

typedef int StateId;
class PinocchioRobotSystem;

class StateMachine {
public:
  StateMachine(StateId state_id, PinocchioRobotSystem *robot)
      : state_id_(state_id), robot_(robot), state_machine_start_time_(0.),
        state_machine_time_(0.), end_time_(0.) {}
  virtual ~StateMachine() = default;

  virtual void FirstVisit() = 0;
  virtual void OneStep() = 0;
  virtual void LastVisit() = 0;
  virtual bool EndOfState() = 0;

  virtual StateId GetNextState() = 0;

  virtual void SetParameters(const YAML::Node &node) = 0;

  StateId State() const { return this->state_id_; }

protected:
  StateId state_id_;
  PinocchioRobotSystem *robot_;
  double state_machine_start_time_;
  double state_machine_time_;
  double end_time_;
};

class Background {
public:
  Background(StateId state_id, PinocchioRobotSystem *robot)
      : state_id_(state_id), robot_(robot), state_machine_start_time_(0.),
        state_machine_time_(0.), end_time_(0.) {}
  virtual ~StateMachine() = default;

  virtual void FirstVisit() = 0;
  virtual void OneStep() = 0;
  virtual void LastVisit() = 0;
  virtual bool EndOfState() = 0;

  virtual StateId GetNextState() = 0;

  virtual void SetParameters(const YAML::Node &node) = 0;

  StateId State() const { return this->state_id_; }

protected:
  StateId state_id_;
  PinocchioRobotSystem *robot_;
  double background_start_time_;
  double background_time_;
  double background_end_time_;
  double moving_duration_;
};

#pragma once
#include "util/util.hpp"

typedef int StateId;
class PinocchioRobotSystem;

class StateMachine {
public:
  StateMachine(StateId state_id, PinocchioRobotSystem *robot)
      : state_id_(state_id), robot_(robot), state_machine_start_time_(0.),
        state_machine_time_(0.), end_time_(0.), rf_end_time_(0.) {}
  virtual ~StateMachine() = default;

  virtual void FirstVisit() = 0;
  virtual void OneStep() = 0;
  virtual void LastVisit() = 0;
  virtual bool EndOfState() = 0;

  virtual StateId GetNextState() = 0;

  virtual void SetParameters(const YAML::Node &node) = 0;
  //alip states
  virtual bool SwitchLeg(){return 0;}
  virtual int GetStance_leg(){return 0;}
  //
  virtual void Reset() {return ;};

  StateId State() const { return this->state_id_; }

protected:
  StateId state_id_;
  PinocchioRobotSystem *robot_;
  double state_machine_start_time_;
  double state_machine_time_;
  double end_time_;
  double rf_end_time_;
};

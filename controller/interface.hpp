#pragma once
#include "controller/interrupt.hpp"

class PinocchioRobotSystem;
class ControlArchitecture;

class Interface {
public:
  Interface() : count_(0), running_time_(0.){};

  virtual ~Interface() = default;

  virtual void GetCommand(void *sensor_data, void *command) = 0;

  Interrupt *interrupt_;

protected:
  int count_;
  double running_time_;

  PinocchioRobotSystem *robot_;
  ControlArchitecture *ctrl_arch_;
};

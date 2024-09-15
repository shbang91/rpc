#pragma once
#include "util/util.hpp"

class PinocchioRobotSystem;
class ControlArchitecture;
class InterruptHandler;

class Interface {
public:
  Interface() : count_(0), running_time_(0.){};

  virtual ~Interface() = default;

  virtual void GetCommand(void *sensor_data, void *command) = 0;

  PinocchioRobotSystem *GetPinocchioModel() { return robot_; }
  InterruptHandler *interrupt_handler_;

protected:
  int count_;
  double running_time_;

  virtual void _SetParameters() = 0;

  PinocchioRobotSystem *robot_;
  ControlArchitecture *ctrl_arch_;
  YAML::Node cfg_;
};

#pragma once
#include <Eigen/Dense>
#include <iostream>

class RobotSystem;
class ControlArchitecture;

class Interface {

public:
  Interface() {
    count_ = 0;
    running_time_ = 0.;
  };

  virtual ~Interface(){};

  virtual void GetCommand(void *_sensor_data, void *_command_data) = 0;

protected:
  int count_;
  double running_time_;

  RobotSystem *robot_;
  ControlArchitecture *control_architecture_;
};

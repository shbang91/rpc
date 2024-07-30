#pragma once
#include "configuration.hpp"

class PinocchioRobotSystem;
class OptimoStateProvider;
class OptimoSensorData;

class OptimoStateEstimator {
public:
  OptimoStateEstimator(PinocchioRobotSystem *robot);
  virtual ~OptimoStateEstimator() = default;

  void Update(OptimoSensorData *sensor_data);

private:
  PinocchioRobotSystem *robot_;
  OptimoStateProvider *sp_;

  // bool b_first_visit_;
};

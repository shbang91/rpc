#pragma once
#include "configuration.hpp"

class PinocchioRobotSystem;
class Med7StateProvider;
class Med7SensorData;

class Med7StateEstimator {
public:
  Med7StateEstimator(PinocchioRobotSystem *robot);
  virtual ~Med7StateEstimator() = default;

  void Update(Med7SensorData *sensor_data);

private:
  PinocchioRobotSystem *robot_;
  Med7StateProvider *sp_;

  // bool b_first_visit_;
};

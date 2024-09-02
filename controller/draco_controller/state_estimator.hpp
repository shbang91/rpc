#pragma once

class DracoSensorData;
class PinocchioRobotSystem;

class StateEstimator {
public:
  StateEstimator(PinocchioRobotSystem *robot) : robot_(robot) {};
  virtual ~StateEstimator() = default;

  virtual void Initialize(DracoSensorData *sensor_Data) = 0;
  virtual void Update(DracoSensorData *sensor_Data) = 0;

  // simulation only
  virtual void UpdateGroundTruthSensorData(DracoSensorData *sensor_data) = 0;

protected:
  PinocchioRobotSystem *robot_;
};

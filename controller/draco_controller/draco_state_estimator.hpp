#pragma once
#include <Eigen/Dense>

class DracoSensorData;
class PinocchioRobotSystem;
class DracoStateProvider;
class SimpleMovingAverage; // filter

class DracoStateEstimator {
public:
  DracoStateEstimator(PinocchioRobotSystem *robot);
  virtual ~DracoStateEstimator() = default;

  void InitializeSensorData(DracoSensorData *sensor_data);
  void UpdateSensorData(DracoSensorData *sensor_data);

  // simulation only
  void UpdateGroundTruthSensorData(DracoSensorData *sensor_data);

private:
  void _ComputeDCM();

  PinocchioRobotSystem *robot_;
  DracoStateProvider *sp_;

  Eigen::Matrix3d R_imu_base_com_;

  Eigen::Vector3d global_leg_odometry_;
  Eigen::Vector3d prev_base_joint_pos_;

  bool b_first_visit_;

  std::vector<SimpleMovingAverage *> com_vel_filter_;
  bool b_sim_;
};

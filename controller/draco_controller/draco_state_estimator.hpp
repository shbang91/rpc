#pragma once
#include "configuration.hpp"

#if B_USE_MATLOGGER
#include <matlogger2/matlogger2.h>
#endif

#include <Eigen/Dense>
#include <vector>

namespace com_vel_filter {
constexpr int kMovingAverage = 0;
constexpr int kExponentialSmoother = 1;
constexpr int kLowPassFilter = 2;
} // namespace com_vel_filter

class DracoSensorData;
class PinocchioRobotSystem;
class DracoStateProvider;
class SimpleMovingAverage;
class ExponentialMovingAverageFilter;
class LowPassVelocityFilter;

class DracoStateEstimator {
public:
  DracoStateEstimator(PinocchioRobotSystem *robot);
  virtual ~DracoStateEstimator();

  void Initialize(DracoSensorData *sensor_data);
  void Update(DracoSensorData *sensor_data);

  // simulation only
  void UpdateGroundTruthSensorData(DracoSensorData *sensor_data);
  void UpdateFootContact(DracoSensorData *sensor_data);

  void GetRlpolicy(DracoSensorData *sensor_data);
  void UpdateWbcObs();

  void Reset();

private:
  void _ComputeDCM();

  PinocchioRobotSystem *robot_;
  DracoStateProvider *sp_;

  Eigen::Matrix3d R_imu_base_com_;

  Eigen::Vector3d global_leg_odometry_;
  Eigen::Vector3d prev_base_joint_pos_;

  bool b_first_visit_;
  bool b_lp_first_visit_ = true;

  int com_vel_filter_type_;
  std::vector<SimpleMovingAverage *> com_vel_mv_avg_filter_;
  ExponentialMovingAverageFilter *com_vel_exp_filter_;
  LowPassVelocityFilter *com_vel_lp_filter_;

#if B_USE_MATLOGGER
  XBot::MatLogger2::Ptr logger_;
#endif
};

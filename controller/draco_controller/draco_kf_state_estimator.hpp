#pragma once
/**
 * Linear KF State Estimator
 *
 * Approach taken from second method in:
 *
 * Flayols, T., Del Prete, A., Wensing, P., Mifsud, A., Benallegue,
 * M. and Stasse, O., 2017, November. Experimental evaluation of simple
 * estimators for humanoid robots. In 2017 IEEE-RAS 17th International
 * Conference on Humanoid Robotics (Humanoids) (pp. 889-895). IEEE.
 */

#include <Eigen/Dense>
#include <vector>

#include "configuration.hpp"
#include "controller/draco_controller/draco_interface.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/state_estimator.hpp"
#include "controller/filter/digital_filters.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "util/util.hpp"

// kalman filter files
#include "controller/state_estimator/FloatingBaseSystemModel.hpp"
#include "controller/state_estimator/MARGFilter.hpp"
#include "controller/state_estimator/PoseMeasurementModel.hpp"
#include "controller/whole_body_controller/managers/contact_detection_manager.hpp"
#include "third_party/kalman_filters/ExtendedKalmanFilter.hpp"
//#include
//"controller/whole_body_controller/managers/contact_detection_manager.hpp"

#if B_USE_MATLOGGER
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#endif

class DracoKFStateEstimator : public StateEstimator {
public:
  enum SupportState { LEFT, RIGHT, DOUBLE };

  DracoKFStateEstimator(PinocchioRobotSystem *robot, const YAML::Node &cfg);
  ~DracoKFStateEstimator();

  void Initialize(DracoSensorData *sensor_data) override;
  void Update(DracoSensorData *sensor_data) override;

  // simulation only
  void UpdateGroundTruthSensorData(DracoSensorData *sensor_data) override;

  void ComputeDCM();

private:
  void updateSupportState(DracoStateProvider *sp, SupportState &support_state);
  Eigen::Matrix3d
  compute_world_to_base_rot(DracoSensorData *data,
                            const Eigen::Matrix3d &rot_world_to_imu,
                            bool use_marg_filter);

protected:
  DracoStateProvider *sp_;
  SupportState current_support_state_;
  SupportState prev_support_state_;
  Eigen::Vector3d prev_base_com_pos_;
  Eigen::Vector3d foot_pos_from_base_pre_transition;
  Eigen::Vector3d foot_pos_from_base_post_transition;

  Eigen::Isometry3d iso_imu_to_base_com_;
  Eigen::Quaterniond quat_imu_to_base_com_;
  Eigen::Vector3d global_linear_offset_;

  // stuff needed for the kalman filter
  State x_hat_;
  Control accelerometer_input_;
  FloatingBaseSystemModel system_model_;
  PoseMeasurement base_estimate_;
  PoseMeasurementModel base_pose_model_;
  Kalman::ExtendedKalmanFilter<State> kalman_filter_;
  MARGFilter margFilter_;
  Eigen::Vector3d base_acceleration_;
  Eigen::Vector3d grav_vec_3D_;

  Eigen::Matrix3d rot_world_to_base;

  //  std::vector<SimpleMovingAverage> cam_filter_;
  std::vector<SimpleMovingAverage> base_accel_filter_;
  std::vector<SimpleMovingAverage> imu_ang_vel_filter_;
  FirstOrderLowPassFilter *base_accel_filt_;
  FirstOrderLowPassFilter *imu_ang_vel_filt_;
  LowPassVelocityFilter *com_vel_filt_;

  // contact sensing
  // std::unique_ptr<ContactDetectionManager> contact_manager_;

  bool b_first_visit_;
  bool b_skip_prediction_;
  bool b_use_marg_filter_;
  bool b_request_offset_reset_;

  int est_ref_foot_frame_;
  int est_non_ref_foot_frame_;

#if B_USE_MATLOGGER
  XBot::MatLogger2::Ptr logger_;
  XBot::MatAppender::Ptr appender_;
#endif
};

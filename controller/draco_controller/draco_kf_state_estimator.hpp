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

#include <vector>
#include <Eigen/Dense>

#include "configuration.hpp"
#include "filters/digital_filters.hpp"
#include "state_estimator/humanoid_state_estimator.hpp"
#include "util/util.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "state_estimator/MARGFilter.hpp"
#include "controller/draco_controller/draco_interface.hpp"
#include "controller/draco_controller/draco_data_manager.hpp"

// kalman filter files
#include "state_estimator/FloatingBaseSystemModel.hpp"
#include "state_estimator/PoseMeasurementModel.hpp"
#include "third_party/kalman_filters/ExtendedKalmanFilter.hpp"

class DracoSensorData;

class DracoKFStateEstimator {
public:
  enum SupportState {LEFT, RIGHT, DOUBLE};

  DracoKFStateEstimator(PinocchioRobotSystem *robot);
  ~DracoKFStateEstimator();

  void initialize(DracoSensorData *data);
  void update(DracoSensorData *data);

private:
  void updateSupportState(DracoStateProvider* sp, SupportState& support_state);
  Eigen::Matrix3d compute_world_to_base_rot(DracoSensorData *data,
                                            Eigen::Matrix3d rot_world_to_imu,
                                            bool use_marg_filter);

protected:
  PinocchioRobotSystem *robot_;
  DracoStateProvider *sp_;
  SupportState current_support_state_;
  SupportState prev_support_state_;
  Eigen::Vector3d foot_pos_from_base_pre_transition;
  Eigen::Vector3d foot_pos_from_base_post_transition;

  Eigen::Isometry3d iso_imu_to_base_com_;
  Eigen::Vector3d global_linear_offset_;

  // stuff needed for the kalman filter
  State x_hat_;
  Control accelerometer_input_;
  FloatingBaseSystemModel system_model_;
  PoseMeasurement base_estimate_;
  PoseMeasurementModel base_pose_model_;
  Kalman::ExtendedKalmanFilter<State> kalman_filter_;
  MARGFilter margFilter_;

  Eigen::Matrix3d rot_world_to_base;

  bool b_first_visit_;
  bool b_skip_prediction;
  bool b_use_marg_filter;
};

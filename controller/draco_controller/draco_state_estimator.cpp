#include "controller/draco_controller/draco_state_estimator.hpp"
#include "controller/draco_controller/draco_data_manager.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_interface.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "util/util.hpp"

DracoStateEstimator::DracoStateEstimator(PinocchioRobotSystem *robot)
    : robot_(robot), R_imu_base_com_(Eigen::Matrix3d::Identity()),
      global_leg_odometry_(Eigen::Vector3d::Zero()),
      prev_base_joint_pos_(Eigen::Vector3d::Zero()), b_first_visit_(true) {
  util::PrettyConstructor(1, "DracoStateEstimator");
  sp_ = DracoStateProvider::GetStateProvider();

  R_imu_base_com_ =
      robot_->GetLinkIsometry(draco_link::torso_imu).linear().transpose() *
      robot_->GetLinkIsometry(draco_link::torso_com_link).linear();
}

void DracoStateEstimator::InitializeSensorData(DracoSensorData *sensor_data) {
  this->UpdateSensorData(sensor_data);
}

void DracoStateEstimator::UpdateSensorData(DracoSensorData *sensor_data) {

  // Estimate floating base orientation
  Eigen::Quaterniond imu_quat(
      sensor_data->imu_frame_quat_[3], sensor_data->imu_frame_quat_[0],
      sensor_data->imu_frame_quat_[1], sensor_data->imu_frame_quat_[2]);
  Eigen::Matrix3d base_joint_ori =
      imu_quat.normalized().toRotationMatrix() * R_imu_base_com_;

  // Update robot model only with base orientation
  robot_->UpdateRobotModel(
      Eigen::Vector3d::Zero(), Eigen::Quaterniond(base_joint_ori).normalized(),
      Eigen::Vector3d::Zero(), sensor_data->imu_ang_vel_,
      sensor_data->joint_pos_, sensor_data->joint_vel_, true);

  // Estimate floating base position
  // anchor frame depending on the stance foot
  Eigen::Vector3d anchor_frame_pos =
      robot_->GetLinkIsometry(sp_->stance_foot_).translation();
  Eigen::Vector3d anchor_frame_vel =
      robot_->GetLinkSpatialVel(sp_->stance_foot_).tail(3);
  if (sp_->stance_foot_ != sp_->prev_stance_foot_) {
    Eigen::Vector3d anchor_frame_pos_diff =
        anchor_frame_pos -
        robot_->GetLinkIsometry(sp_->prev_stance_foot_).translation();
    global_leg_odometry_ += anchor_frame_pos_diff;
  }

  // estimate position
  Eigen::Vector3d base_joint_pos = global_leg_odometry_ - anchor_frame_pos;

  if (b_first_visit_) {
    prev_base_joint_pos_ = base_joint_pos;
    b_first_visit_ = false;
  }

  // estimate base linear velocity
  Eigen::Vector3d base_joint_lin_vel =
      (base_joint_pos - prev_base_joint_pos_) / sp_->servo_dt_;

  // save current time step data
  sp_->prev_stance_foot_ = sp_->stance_foot_;
  prev_base_joint_pos_ = base_joint_pos;

  // Update robot model with full estimate
  robot_->UpdateRobotModel(
      base_joint_pos, Eigen::Quaterniond(base_joint_ori).normalized(),
      base_joint_lin_vel, sensor_data->imu_ang_vel_, sensor_data->joint_pos_,
      sensor_data->joint_vel_, true);

  // foot contact switch
  sp_->b_lf_contact_ = (sensor_data->b_lf_contact_) ? true : false;
  sp_->b_rf_contact_ = (sensor_data->b_rf_contact_) ? true : false;

  // TODO:velocity filtering for com vel (for real experiment)

  // Save estimated base joint states
  DracoDataManager *dm = DracoDataManager::GetDataManager();
  dm->data_->est_base_joint_pos_ = base_joint_pos;
  Eigen::Quaterniond base_joint_quat(base_joint_ori);
  dm->data_->est_base_joint_ori_ << base_joint_quat.normalized().coeffs();
  dm->data_->est_base_joint_lin_vel_ = base_joint_lin_vel;
  dm->data_->est_base_joint_ang_vel_ = sensor_data->imu_ang_vel_;

  dm->data_->base_joint_pos_ = sensor_data->base_joint_pos_;
  dm->data_->base_joint_ori_ = sensor_data->base_joint_quat_;
  dm->data_->base_joint_lin_vel_ = sensor_data->base_joint_lin_vel_;
  dm->data_->base_joint_ang_vel_ = sensor_data->base_joint_ang_vel_;

  // compute dcm
  this->_ComputeDCM();
}

void DracoStateEstimator::_ComputeDCM() {
  Eigen::Vector3d com_pos = robot_->GetRobotComPos();
  Eigen::Vector3d com_vel = robot_->GetRobotComLinVel();
  double omega = sqrt(9.81 / com_pos[2]);

  sp_->prev_dcm_ = sp_->dcm_;
  sp_->dcm_ = com_pos + com_vel / omega;

  double cutoff_period =
      0.01; // 10ms cut-off period for first-order low pass filter
  double alpha = sp_->servo_dt_ / cutoff_period;

  sp_->dcm_vel_ = alpha * (sp_->dcm_ - sp_->prev_dcm_) / sp_->servo_dt_ +
                  (1 - alpha) * sp_->dcm_vel_;
}

void DracoStateEstimator::UpdateGroundTruthSensorData(
    DracoSensorData *sensor_data) {
  Eigen::Vector4d base_joint_ori = sensor_data->base_joint_quat_;
  Eigen::Quaterniond base_joint_quat(base_joint_ori[3], base_joint_ori[0],
                                     base_joint_ori[1], base_joint_ori[2]);

  robot_->UpdateRobotModel(
      sensor_data->base_joint_pos_, base_joint_quat.normalized(),
      sensor_data->base_joint_lin_vel_, sensor_data->base_joint_ang_vel_,
      sensor_data->joint_pos_, sensor_data->joint_vel_, true);

  this->_ComputeDCM();

  DracoDataManager *dm = DracoDataManager::GetDataManager();
  dm->data_->base_joint_pos_ = sensor_data->base_joint_pos_;
  dm->data_->base_joint_ori_ = sensor_data->base_joint_quat_;
  dm->data_->base_joint_lin_vel_ = sensor_data->base_joint_lin_vel_;
  dm->data_->base_joint_ang_vel_ = sensor_data->base_joint_ang_vel_;

  dm->data_->joint_positions_ = sensor_data->joint_pos_;
}

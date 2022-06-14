#include "controller/draco_controller/draco_state_estimator.hpp"
#include "controller/draco_controller/draco_data_manager.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_interface.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "util/util.hpp"

DracoStateEstimator::DracoStateEstimator(PinocchioRobotSystem *robot) {
  util::PrettyConstructor(1, "DracoStateEstimator");
  robot_ = robot;

  R_imu_base_com_.setZero();
  global_leg_odometry_.setZero();
  prev_base_joint_pos_.setZero();

  sp_ = DracoStateProvider::GetStateProvider();
}

void DracoStateEstimator::InitializeSensorData(DracoSensorData *sensor_data) {
  this->UpdateSensorData(sensor_data);
}

void DracoStateEstimator::UpdateSensorData(DracoSensorData *sensor_data) {

  // Estimate floating base orientation
  static bool b_first_visit(true);
  if (b_first_visit) {
    R_imu_base_com_ =
        robot_->GetLinkIsometry(draco_link::torso_imu).linear().transpose() *
        robot_->GetLinkIsometry(draco_link::torso_com_link).linear();
    b_first_visit = false;
  }

  Eigen::Quaterniond imu_quat(
      sensor_data->imu_frame_quat_(3), sensor_data->imu_frame_quat_(0),
      sensor_data->imu_frame_quat_(1), sensor_data->imu_frame_quat_(2));
  Eigen::Matrix3d base_joint_ori =
      imu_quat.toRotationMatrix() * R_imu_base_com_;

  // Update robot model only with base orientation
  robot_->UpdateRobotModel(
      Eigen::Vector3d::Zero(), Eigen::Quaterniond(base_joint_ori),
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

  static bool first_visit2(true);
  if (first_visit2) {
    prev_base_joint_pos_ = base_joint_pos;
    first_visit2 = false;
  }

  // estimate base linear velocity
  Eigen::Vector3d base_joint_lin_vel =
      (base_joint_pos - prev_base_joint_pos_) / sp_->servo_dt_;

  // save current time step data
  sp_->prev_stance_foot_ = sp_->stance_foot_;
  prev_base_joint_pos_ = base_joint_pos;

  // Update robot model with full estimate
  robot_->UpdateRobotModel(base_joint_pos, Eigen::Quaterniond(base_joint_ori),
                           base_joint_lin_vel, sensor_data->imu_ang_vel_,
                           sensor_data->joint_pos_, sensor_data->joint_vel_,
                           true);

  // foot contact switch
  sp_->b_lf_contact_ = (sensor_data->b_lf_contact_) ? true : false;
  sp_->b_rf_contact_ = (sensor_data->b_rf_contact_) ? true : false;

  // TODO:velocity filtering for com vel (for real experiment)

  // Save estimated base joint states
  DracoDataManager::GetDataManager()->data_->est_base_joint_pos_ =
      base_joint_pos;
  Eigen::Quaterniond base_joint_quat(base_joint_ori);
  DracoDataManager::GetDataManager()->data_->est_base_joint_ori_
      << base_joint_quat.x(),
      base_joint_quat.y(), base_joint_quat.z(), base_joint_quat.w();
  DracoDataManager::GetDataManager()->data_->est_base_joint_lin_vel_ =
      base_joint_lin_vel;
  DracoDataManager::GetDataManager()->data_->est_base_joint_ang_vel_ =
      sensor_data->imu_ang_vel_;

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
  Eigen::Quaterniond base_joint_quat(base_joint_ori(3), base_joint_ori(0),
                                     base_joint_ori(1), base_joint_ori(2));

  robot_->UpdateRobotModel(
      sensor_data->base_joint_pos_, base_joint_quat,
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

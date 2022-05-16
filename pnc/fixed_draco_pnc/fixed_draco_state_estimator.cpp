#include "pnc/fixed_draco_pnc/fixed_draco_state_estimator.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_data_manager.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"
#include "pnc/robot_system/robot_system.hpp"
#include "util/util.hpp"

FixedDracoStateEstimator::FixedDracoStateEstimator(RobotSystem *_robot) {
  util::PrettyConstructor(1, "FixedDracoStateEstimator");
  robot_ = _robot;
  sp_ = FixedDracoStateProvider::GetStateProvider();

  base_joint_orientation_.setIdentity();
}

FixedDracoStateEstimator::~FixedDracoStateEstimator(){};

void FixedDracoStateEstimator::UpdateModelWithGroundTruth(
    FixedDracoSensorData *_sensor_data) {
  Eigen::Vector4d base_com_quat = _sensor_data->base_com_quat_;
  Eigen::Quaternion<double> base_com_ori(base_com_quat[3], base_com_quat[0],
                                         base_com_quat[1], base_com_quat[2]);
  Eigen::Vector4d base_joint_quat = _sensor_data->base_joint_quat_;
  Eigen::Quaternion<double> base_joint_ori(
      base_joint_quat[3], base_joint_quat[0], base_joint_quat[1],
      base_joint_quat[2]);

  robot_->UpdateRobotModel(
      _sensor_data->base_com_pos_, base_com_ori,
      _sensor_data->base_com_lin_vel_, _sensor_data->base_com_ang_vel_,
      _sensor_data->base_joint_pos_, base_joint_ori,
      _sensor_data->base_joint_lin_vel_, _sensor_data->base_joint_ang_vel_,
      _sensor_data->joint_positions_, _sensor_data->joint_velocities_, false);

  FixedDracoDataManager *dm = FixedDracoDataManager::GetDataManager();
  dm->data_->base_com_pos_ = _sensor_data->base_com_pos_;
  dm->data_->base_com_ori_ = base_com_quat;
  dm->data_->base_com_lin_vel_ = _sensor_data->base_com_lin_vel_;
  dm->data_->base_com_ang_vel_ = _sensor_data->base_com_ang_vel_;

  dm->data_->base_joint_pos_ = _sensor_data->base_joint_pos_;
  dm->data_->base_joint_ori_ = base_joint_quat;
  dm->data_->base_joint_lin_vel_ = _sensor_data->base_joint_lin_vel_;
  dm->data_->base_joint_ang_vel_ = _sensor_data->base_joint_ang_vel_;

  // for meshcat visualizing
  dm->data_->joint_positions_ = robot_->joint_positions_;
}

void FixedDracoStateEstimator::InitializeModel(
    FixedDracoSensorData *_sensor_data) {
  this->UpdateModel(_sensor_data);
}

void FixedDracoStateEstimator::UpdateModel(FixedDracoSensorData *_sensor_data) {

  // Estimate floating base orientation
  this->EstimateOrientation(_sensor_data);

  // Update robot model only with base orientation
  robot_->UpdateRobotModel(
      Eigen::Vector3d::Zero(),
      Eigen::Quaternion<double>(base_joint_orientation_),
      Eigen::Vector3d::Zero(), _sensor_data->imu_frame_velocities_.head(3),
      Eigen::Vector3d::Zero(),
      Eigen::Quaternion<double>(base_joint_orientation_),
      Eigen::Vector3d::Zero(), _sensor_data->imu_frame_velocities_.head(3),
      _sensor_data->joint_positions_, _sensor_data->joint_velocities_, false);

  // Debugging purpose
  // std::cout << "===================" << std::endl;
  // std::cout << "======PnC Robot====" << std::endl;
  // std::cout << "===================" << std::endl;
  // std::cout << "base com pos" << std::endl;
  // std::cout << base_com_position_ << std::endl;
  // std::cout << "base com ori" << std::endl;
  // std::cout << robot_->GetLinkIso("torso_com_link").linear() << std::endl;
  // std::cout << "base joint pos" << std::endl;
  // std::cout << base_joint_position_ << std::endl;
  // std::cout << "base joint ori" << std::endl;
  // std::cout << base_joint_orientation_ << std::endl;
  // std::cout << "base_com_lin_vel" << std::endl;
  // std::cout << base_com_lin_vel_ << std::endl;
  // std::cout << "base_com_ang_vel" << std::endl;
  // std::cout << robot_->GetLinkVel("torso_com_link").head(3) << std::endl;
  // std::cout << "base_joint_lin_vel" << std::endl;
  // std::cout << base_joint_lin_vel_ << std::endl;
  // std::cout << "base_joint_ang_vel" << std::endl;
  // std::cout << robot_->GetLinkVel("torso_com_link").head(3) << std::endl;
  // std::cout << "imu_frame_ang_vel" << std::endl;
  // std::cout << robot_->GetLinkVel("torso_imu").head(3) << std::endl;
  // std::cout << "imu_frame_lin_vel" << std::endl;
  // std::cout << robot_->GetLinkVel("torso_imu").tail(3) << std::endl;
  // std::cout << "lf pos" << std::endl;
  // std::cout << robot_->GetLinkIso("l_foot_contact").translation() <<
  // std::endl; std::cout << "rf pos" << std::endl; std::cout <<
  // robot_->GetLinkIso("r_foot_contact").translation() << std::endl;
}

void FixedDracoStateEstimator::EstimateOrientation(
    FixedDracoSensorData *_sensor_data) {
  Eigen::Matrix3d R_0_imu = robot_->GetLinkIso("torso_imu").linear();
  Eigen::Matrix3d R_0_base_com = robot_->GetLinkIso("torso_com_link").linear();
  Eigen::Matrix3d R_imu_base_com = R_0_imu.transpose() * R_0_base_com;

  // estimate orientation
  base_joint_orientation_ =
      _sensor_data->imu_frame_isometry_.block(0, 0, 3, 3) * R_imu_base_com;
}

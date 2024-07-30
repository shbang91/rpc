#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/optimo_controller/optimo_interface.hpp"
#include "controller/optimo_controller/optimo_state_estimator.hpp"
#include "controller/optimo_controller/optimo_state_provider.hpp"

#include "util/util.hpp"

OptimoStateEstimator::OptimoStateEstimator(PinocchioRobotSystem *robot)
    : robot_(robot) {
  util::PrettyConstructor(1, "OptimoStateEstimator");
  sp_ = OptimoStateProvider::GetStateProvider();
}

void OptimoStateEstimator::Update(OptimoSensorData *sensor_data) {
  const Eigen::Vector4d &base_quat_vec = sensor_data->base_quat_;
  Eigen::Quaterniond base_quat(base_quat_vec[3], base_quat_vec[0],
                               base_quat_vec[1], base_quat_vec[2]);
  robot_->UpdateRobotModel(sensor_data->base_pos_, base_quat,
                           Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
                           sensor_data->joint_pos_, sensor_data->joint_vel_,
                           false);
}

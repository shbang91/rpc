#pragma once

#include <Eigen/Dense>
#include <memory>

#include "controller/interface.hpp"
#include "controller/optimo_controller/optimo_definition.hpp"

class OptimoSensorData {
public:
  OptimoSensorData() {
    joint_pos_ =(Eigen::VectorXd::Zero(optimo::n_adof));
    Eigen::VectorXd::Zero(7);
    joint_vel_ = Eigen::VectorXd::Zero(7);
    joint_trq_ = Eigen::VectorXd::Zero(7); 
    joint_sea_trq_ = Eigen::VectorXd::Zero(optimo::n_adof);
    base_joint_pos_ = Eigen::Vector3d::Zero();
    base_joint_quat_ = Eigen::Vector4d(0., 0., 0., 1.);
  }
  virtual ~OptimoSensorData() = default;

  Eigen::VectorXd joint_pos_;
  Eigen::VectorXd joint_vel_;
  Eigen::VectorXd joint_trq_;
  Eigen::VectorXd joint_sea_trq_;
  
  Eigen::Vector3d base_joint_pos_;
  Eigen::Vector4d base_joint_quat_;
};

class OptimoCommand {
public:
  OptimoCommand() {
    joint_pos_cmd_ = Eigen::VectorXd::Zero(optimo::n_adof);
    joint_vel_cmd_ = Eigen::VectorXd::Zero(optimo::n_adof);
    joint_trq_cmd_ = Eigen::VectorXd::Zero(optimo::n_adof);
  }
  ~OptimoCommand() = default;

  Eigen::VectorXd joint_pos_cmd_;
  Eigen::VectorXd joint_vel_cmd_;
  Eigen::VectorXd joint_trq_cmd_;
};

class OptimoInterface : public Interface {
public:
  OptimoInterface();
  virtual ~OptimoInterface();

  void GetCommand(void *sensor_data, void *command_data) override;
};

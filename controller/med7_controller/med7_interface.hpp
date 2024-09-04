#pragma once

#include <Eigen/Dense>
#include <memory>

#include "controller/interface.hpp"
#include "controller/med7_controller/med7_definition.hpp"

class Med7StateProvider;
class Med7TaskGainHandler;
class Med7StateEstimator;

class Med7SensorData {
public:
  Med7SensorData()
      : joint_pos_(Eigen::VectorXd::Zero(med7::n_adof)),
        joint_vel_(Eigen::VectorXd::Zero(med7::n_adof)),
        joint_trq_(Eigen::VectorXd::Zero(med7::n_adof)),
        joint_sea_trq_(Eigen::VectorXd::Zero(med7::n_adof)),
        base_pos_(Eigen::Vector3d::Zero()),
        base_quat_(Eigen::Vector4d(0., 0., 0., 1.)){};

  virtual ~Med7SensorData() = default;

  Eigen::VectorXd joint_pos_;
  Eigen::VectorXd joint_vel_;
  Eigen::VectorXd joint_trq_;
  Eigen::VectorXd joint_sea_trq_;

  Eigen::Vector3d base_pos_;
  Eigen::Vector4d base_quat_;
};

class Med7Command {
public:
  Med7Command()
      : joint_pos_cmd_(Eigen::VectorXd::Zero(med7::n_adof)),
        joint_vel_cmd_(Eigen::VectorXd::Zero(med7::n_adof)),
        joint_trq_cmd_(Eigen::VectorXd::Zero(med7::n_adof)){};
  virtual ~Med7Command() = default;

  Eigen::VectorXd joint_pos_cmd_;
  Eigen::VectorXd joint_vel_cmd_;
  Eigen::VectorXd joint_trq_cmd_;
};

class Med7Interface : public Interface {
public:
  Med7Interface();
  virtual ~Med7Interface();

  void GetCommand(void *sensor_data, void *command_data) override;

  Med7TaskGainHandler *task_gain_handler_;

private:
  Med7StateProvider *sp_;
  Med7StateEstimator *se_;
  void _SafeCommand(Med7SensorData *data, Med7Command *command);
};

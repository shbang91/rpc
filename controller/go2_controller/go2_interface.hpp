#pragma once

#include <Eigen/Dense>
#include <memory>

#include "controller/go2_controller/go2_definition.hpp"
#include "controller/interface.hpp"

class StateEstimator;

class Go2SensorData {
public:
  Go2SensorData()
      : imu_frame_quat_(0, 0, 0, 1), imu_ang_vel_(Eigen::Vector3d::Zero()),
        imu_dvel_(Eigen::Vector3d::Zero()),
        imu_lin_acc_(Eigen::Vector3d::Zero()),
        joint_pos_(Eigen::VectorXd::Zero(go2::n_adof)),
        joint_vel_(Eigen::VectorXd::Zero(go2::n_adof)),
        b_FL_foot_contact_(false), b_FR_foot_contact_(false),
        b_RL_foot_contact_(false), b_RR_foot_contact_(false),
        FL_normal_force_(0.), FR_normal_force_(0.), RL_normal_force_(0.),
        RR_normal_force_(0.), base_joint_pos_(Eigen::Vector3d::Zero()),
        base_joint_quat_(0, 0, 0, 1),
        base_joint_lin_vel_(Eigen::Vector3d::Zero()),
        base_joint_ang_vel_(Eigen::Vector3d::Zero()){};
  virtual ~Go2SensorData() = default;

  Eigen::Vector4d imu_frame_quat_; // x, y, z, w order
  Eigen::Vector3d imu_ang_vel_;    // in world frame
  Eigen::Vector3d imu_dvel_;       // in world frame
  Eigen::Vector3d imu_lin_acc_;    // imu_dvel_ / dt_
  Eigen::VectorXd joint_pos_;
  Eigen::VectorXd joint_vel_;
  bool b_FL_foot_contact_;
  bool b_FR_foot_contact_;
  bool b_RL_foot_contact_;
  bool b_RR_foot_contact_;
  float FL_normal_force_;
  float FR_normal_force_;
  float RL_normal_force_;
  float RR_normal_force_;

  // Debug or using ground truth state estimator
  Eigen::Vector3d base_joint_pos_;
  Eigen::Vector4d base_joint_quat_; // x, y, z, w order
  Eigen::Vector3d base_joint_lin_vel_;
  Eigen::Vector3d base_joint_ang_vel_;
};

class Go2Command {
public:
  Go2Command()
      : joint_pos_cmd_(Eigen::VectorXd::Zero(go2::n_adof)),
        joint_vel_cmd_(Eigen::VectorXd::Zero(go2::n_adof)),
        joint_trq_cmd_(Eigen::VectorXd::Zero(go2::n_adof)){};
  virtual ~Go2Command() = default;

  Eigen::VectorXd joint_pos_cmd_;
  Eigen::VectorXd joint_vel_cmd_;
  Eigen::VectorXd joint_trq_cmd_;
};

class Go2Interface : public Interface {
public:
  Go2Interface();
  virtual ~Go2Interface();

  void GetCommand(void *sensor_data, void *command_data) override;

  // Go2TaskGainHandler *task_gain_handler_;

private:
  StateEstimator *se_;
  // Go2StateProvider *sp_;
  void _SafeCommand(Go2SensorData *data, Go2Command *command);
  void _SetParameters() override;

  // state estimator selection
  std::string state_estimator_type_;
  std::string wbc_type_;
  bool b_cheater_mode_;
};

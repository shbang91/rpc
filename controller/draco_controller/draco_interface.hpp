#pragma once

#include <Eigen/Dense>
#include <memory>

#include "controller/draco_controller/draco_definition.hpp"
#include "controller/interface.hpp"

class DracoStateEstimator;
class DracoKFStateEstimator;
class DracoStateProvider;
class DracoTaskGainHandler;

class DracoSensorData {
public:
  DracoSensorData()
      : imu_frame_quat_(0, 0, 0, 1), imu_ang_vel_(Eigen::Vector3d::Zero()),
        imu_dvel_(Eigen::Vector3d::Zero()),
        imu_lin_acc_(Eigen::Vector3d::Zero()),
        joint_pos_(Eigen::VectorXd::Zero(draco::n_adof)),
        joint_vel_(Eigen::VectorXd::Zero(draco::n_adof)), b_lf_contact_(false),
        b_rf_contact_(false), lf_contact_normal_(0.), rf_contact_normal_(0.),
        base_joint_pos_(Eigen::Vector3d::Zero()), base_joint_quat_(0, 0, 0, 1),
        base_joint_lin_vel_(Eigen::Vector3d::Zero()),
        base_joint_ang_vel_(Eigen::Vector3d::Zero()),
        res_rl_action_(Eigen::VectorXd::Zero(3)),
        initial_stance_leg_(0),
        MPC_freq_(0),
        policy_command_(Eigen::VectorXd::Zero(3)){};
  virtual ~DracoSensorData() = default;

  Eigen::Vector4d imu_frame_quat_; // x, y, z, w order
  Eigen::Vector3d imu_ang_vel_;
  Eigen::Vector3d imu_dvel_;
  Eigen::Vector3d imu_lin_acc_;
  Eigen::VectorXd joint_pos_;
  Eigen::VectorXd joint_vel_;
  bool b_lf_contact_;
  bool b_rf_contact_;
  float lf_contact_normal_;
  float rf_contact_normal_;

  // Debug or using ground truth state estimator
  Eigen::Vector3d base_joint_pos_;
  Eigen::Vector4d base_joint_quat_; // x, y, z, w order
  Eigen::Vector3d base_joint_lin_vel_;
  Eigen::Vector3d base_joint_ang_vel_;

  //residual swfoot rl action
  Eigen::VectorXd res_rl_action_;
  //Command that rl_action, can be randomized
  Eigen::VectorXd policy_command_;
  int initial_stance_leg_;
  int MPC_freq_;

};

class DracoCommand {
public:
  DracoCommand()
      : joint_pos_cmd_(Eigen::VectorXd::Zero(draco::n_adof)),
        joint_vel_cmd_(Eigen::VectorXd::Zero(draco::n_adof)),
        joint_trq_cmd_(Eigen::VectorXd::Zero(draco::n_adof)),
        wbc_obs_(Eigen::VectorXd::Zero(33)),
        rl_trigger_(false){};
  virtual ~DracoCommand() = default;

  Eigen::VectorXd joint_pos_cmd_;
  Eigen::VectorXd joint_vel_cmd_;
  Eigen::VectorXd joint_trq_cmd_;

  //foot policy rl trained
  Eigen::VectorXd wbc_obs_;
  bool rl_trigger_;
};

class DracoInterface : public Interface {
public:
  DracoInterface();
  virtual ~DracoInterface();

  void GetCommand(void *sensor_data, void *command_data) override;
  void Reset();

private:
  DracoStateEstimator *se_;
  DracoStateProvider *sp_;
  void _SafeCommand(DracoSensorData *data, DracoCommand *command);
};

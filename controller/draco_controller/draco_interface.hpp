#pragma once

#include <Eigen/Dense>
#include <memory>

#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_vr_teleop_manager.hpp"
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
        joint_pos_(Eigen::VectorXd::Zero(draco::n_adof)),
        joint_vel_(Eigen::VectorXd::Zero(draco::n_adof)), b_lf_contact_(false),
        b_rf_contact_(false), base_joint_pos_(Eigen::Vector3d::Zero()),
        base_joint_quat_(0, 0, 0, 1),
        base_joint_lin_vel_(Eigen::Vector3d::Zero()),
        base_joint_ang_vel_(Eigen::Vector3d::Zero()){};
  virtual ~DracoSensorData() = default;

  Eigen::Vector4d imu_frame_quat_; // x, y, z, w order
  Eigen::Vector3d imu_ang_vel_;
  Eigen::Vector3d imu_dvel_;
  Eigen::VectorXd joint_pos_;
  Eigen::VectorXd joint_vel_;
  bool b_lf_contact_;
  bool b_rf_contact_;

  // Debug or using ground truth state estimator
  Eigen::Vector3d base_joint_pos_;
  Eigen::Vector4d base_joint_quat_; // x, y, z, w order
  Eigen::Vector3d base_joint_lin_vel_;
  Eigen::Vector3d base_joint_ang_vel_;
};

class DracoCommand {
public:
  DracoCommand()
      : joint_pos_cmd_(Eigen::VectorXd::Zero(draco::n_adof)),
        joint_vel_cmd_(Eigen::VectorXd::Zero(draco::n_adof)),
        joint_trq_cmd_(Eigen::VectorXd::Zero(draco::n_adof)){};
  virtual ~DracoCommand() = default;

  Eigen::VectorXd joint_pos_cmd_;
  Eigen::VectorXd joint_vel_cmd_;
  Eigen::VectorXd joint_trq_cmd_;
};

class DracoInterface : public Interface {
public:
  DracoInterface();
  virtual ~DracoInterface();

  void GetCommand(void *sensor_data, void *command_data) override;

  DracoTaskGainHandler *task_gain_handler_;

private:
  DracoStateEstimator *se_;
  DracoKFStateEstimator *se_kf_;
  DracoStateProvider *sp_;
  void _SafeCommand(DracoSensorData *data, DracoCommand *command);
  void _ProcessVRInput(DracoVRCommands *commands, void *sensor_data);
};

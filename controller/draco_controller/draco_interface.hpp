#include <Eigen/Dense>
#include <memory>

#include "controller/draco_controller/draco_definition.hpp"
#include "controller/interface.hpp"

class DracoStateEstimator;
class DracoStateProvider;

class DracoSensorData {
public:
  DracoSensorData() {
    imu_frame_quat_.setIdentity();
    imu_ang_vel_.setZero();
    joint_pos_ = Eigen::VectorXd::Zero(draco::n_adof);
    joint_vel_ = Eigen::VectorXd::Zero(draco::n_adof);
    b_lf_contact_ = false;
    b_rf_contact_ = false;

    base_joint_pos_.setZero();
    base_joint_quat_.setZero();
    base_joint_lin_vel_.setZero();
    base_joint_ang_vel_.setZero();
  };
  virtual ~DracoSensorData() = default;

  Eigen::Vector4d imu_frame_quat_; // x, y, z, w order
  Eigen::Vector3d imu_ang_vel_;
  Eigen::VectorXd joint_pos_;
  Eigen::VectorXd joint_vel_;
  bool b_lf_contact_;
  bool b_rf_contact_;

  // Debug
  Eigen::Vector3d base_joint_pos_;
  Eigen::Vector4d base_joint_quat_; // x, y, z, w order
  Eigen::Vector3d base_joint_lin_vel_;
  Eigen::Vector3d base_joint_ang_vel_;
};

class DracoCommand {
public:
  DracoCommand() {
    joint_pos_cmd_ = Eigen::VectorXd::Zero(draco::n_adof);
    joint_vel_cmd_ = Eigen::VectorXd::Zero(draco::n_adof);
    joint_trq_cmd_ = Eigen::VectorXd::Zero(draco::n_adof);
  };
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

private:
  DracoStateEstimator *se_;
  DracoStateProvider *sp_;
  int waiting_count_;
  void _SafeCommand(DracoSensorData *data, DracoCommand *command);
};

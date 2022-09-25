#pragma once

#include "draco.pb.h" //in build/messages/
#include <Eigen/Dense>
#include <zmq.hpp>

class DracoData {
public:
  DracoData();
  ~DracoData() = default;

  double time_;

  Eigen::Vector3d base_joint_pos_;
  Eigen::Vector4d base_joint_ori_;
  Eigen::Vector3d base_joint_lin_vel_;
  Eigen::Vector3d base_joint_ang_vel_;

  Eigen::VectorXd joint_positions_;

  Eigen::Vector3d est_base_joint_pos_;
  Eigen::Vector4d est_base_joint_ori_;
  Eigen::Vector3d est_base_joint_lin_vel_;
  Eigen::Vector3d est_base_joint_ang_vel_;

  Eigen::VectorXd des_com_pos_;
  Eigen::VectorXd act_com_pos_;
  Eigen::VectorXd des_com_vel_;
  Eigen::VectorXd act_com_vel_;

  Eigen::VectorXd des_lf_pos_;
  Eigen::VectorXd act_lf_pos_;
  Eigen::VectorXd des_rf_pos_;
  Eigen::VectorXd act_rf_pos_;

  Eigen::VectorXd des_lf_vel_;
  Eigen::VectorXd act_lf_vel_;
  Eigen::VectorXd des_rf_vel_;
  Eigen::VectorXd act_rf_vel_;

  Eigen::VectorXd des_lf_force_;
  Eigen::VectorXd act_lf_force_;
  Eigen::VectorXd des_rf_force_;
  Eigen::VectorXd act_rf_force_;

  Eigen::VectorXd des_torso_ori_;
  Eigen::VectorXd act_torso_ori_;
  Eigen::VectorXd des_torso_vel_;
  Eigen::VectorXd act_torso_vel_;

  // KF state estimator
  // state estimator
  Eigen::VectorXd base_quat_kf_ = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd base_euler_kf_ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd base_pos_kf_ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd base_vel_kf_ = Eigen::VectorXd::Zero(3);

  Eigen::VectorXd base_com_pos_ = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd base_com_quat_ = Eigen::VectorXd::Zero(4);

  // feet contact status
  bool lfoot_contact_ = false;
  bool rfoot_contact_ = false;
  bool lf_contact_ = false;
  bool rf_contact_ = false;
};

// Singleton class
class DracoDataManager {
public:
  static DracoDataManager *GetDataManager();
  ~DracoDataManager() = default;

  void InitializeSocket(const std::string &ip_address);
  void SendData();

  std::unique_ptr<DracoData> data_;

private:
  DracoDataManager();

  std::unique_ptr<zmq::context_t> context_;
  std::unique_ptr<zmq::socket_t> socket_;

  bool b_initialize_socket_;
};

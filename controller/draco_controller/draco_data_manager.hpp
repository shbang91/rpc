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
  Eigen::VectorXd torques_;

  Eigen::Vector3d est_base_joint_pos_;
  Eigen::Vector4d est_base_joint_ori_;
  Eigen::Vector3d est_base_joint_lin_vel_;
  Eigen::Vector3d est_base_joint_ang_vel_;

  Eigen::VectorXd des_com_pos_;
  Eigen::VectorXd act_com_pos_;
  Eigen::VectorXd des_com_vel_;
  Eigen::VectorXd act_com_vel_;
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

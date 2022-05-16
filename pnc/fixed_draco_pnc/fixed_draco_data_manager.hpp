#pragma once

#include "fixed_draco.pb.h" //in build/messages/
#include <Eigen/Dense>
#include <zmq.hpp>

class FixedDracoData;

// Singleton class
class FixedDracoDataManager {
public:
  static FixedDracoDataManager *GetDataManager();
  ~FixedDracoDataManager();

  void InitializeSocket(const std::string &ip_address);
  void SendData();

  std::unique_ptr<FixedDracoData> data_;

private:
  FixedDracoDataManager();

  std::unique_ptr<zmq::context_t> context_;
  std::unique_ptr<zmq::socket_t> socket_;

  bool b_initialize_socket_;
};

class FixedDracoData {
public:
  FixedDracoData();
  ~FixedDracoData();

  double time_;

  Eigen::Vector3d base_joint_pos_;
  Eigen::Vector4d base_joint_ori_;
  Eigen::Vector3d base_joint_lin_vel_;
  Eigen::Vector3d base_joint_ang_vel_;

  Eigen::Vector3d base_com_pos_;
  Eigen::Vector4d base_com_ori_;
  Eigen::Vector3d base_com_lin_vel_;
  Eigen::Vector3d base_com_ang_vel_;

  Eigen::VectorXd joint_positions_;
};

#pragma once
#include "planner/locomotion/contact_state.hpp"
#include <vector>

class MPCInputData {
public:
  MPCInputData() {
    com_pos_.setZero();
    euler_angle_.setZero();
    com_vel_.setZero();
    euler_angle_rate_.setZero();
    body_inertia_.setZero();
    l_foot_pose_.setZero();
    r_foot_pose_.setZero();

    l_foot_state_.clear();
    r_foot_state_.clear();
  }
  virtual ~MPCInputData() = default;

protected:
  // contact planning info
  std::vector<ContactState> l_foot_state_;
  std::vector<ContactState> r_foot_state_;

  // robot states
  Eigen::Vector3d com_pos_;
  Eigen::Vector3d euler_angle_;
  Eigen::Vector3d com_vel_;
  Eigen::Vector3d euler_angle_rate_;
  Eigen::Matrix<double, 6, 1>
      body_inertia_; // in order of xx, yy, zz, xy, yz, zx
  Eigen::Matrix<double, 7, 1>
      l_foot_pose_; // current left foot pose  q.x, q.y, q.z, q.w, x,y,z,
  Eigen::Matrix<double, 7, 1>
      r_foot_pose_; // current right foot pose q.x, q.y, q.z, q.w, x,y,z

  // mpc problem formulation
  // double T; // mpc prediction horizon time
  // int N;    // number of node

  //  reference traj from planner manager
  Eigen::VectorXd ref_euler_angle_;
  Eigen::VectorXd ref_com_pos_;
  Eigen::VectorXd ref_euler_angle_rate_;
  Eigen::VectorXd ref_com_vel_;
};

class MPCOutputData {
public:
  MPCOutputData() {}
  virtual ~MPCOutputData() = default;

protected:
  // state results
  Eigen::VectorXd euler_angle_;
  Eigen::VectorXd com_pos_;
  Eigen::VectorXd euler_angle_rate_;
  Eigen::VectorXd com_vel_;

  // control results
  Eigen::VectorXd lfoot_rf;
  Eigen::VectorXd rfoot_rf;
};

class PinocchioRobotSystem;

class MPCHandler {
public:
  MPCHandler(PinocchioRobotSystem *robot) { robot_ = robot; }
  virtual ~MPCHandler() = default;

  void SolveMPC() {
    _GetMPCInputData();
    _SendData(); // zmq & protobuf
  }
  bool RecieveSolution() { _GetMPCOutputData(); } // zmq & protobuf

protected:
  PinocchioRobotSystem *robot_;
  MPCInputData *input_data_;
  MPCOutputData *output_data_;

  virtual void _GetMPCInputData() = 0;
  virtual void _GetMPCOutputData() = 0;
  virtual void _SendData() = 0;
};

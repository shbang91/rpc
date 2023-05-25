#pragma once
#include <Eigen/Dense>
#include <iostream>

#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "util/util.hpp"

class Task {
public:
  Task(PinocchioRobotSystem *robot, const int dim)
      : robot_(robot), dim_(dim), target_idx_(0),
        rot_link_w_(Eigen::Matrix3d::Identity()) {

    des_pos_ = Eigen::VectorXd::Zero(dim_);
    des_vel_ = Eigen::VectorXd::Zero(dim_);
    des_acc_ = Eigen::VectorXd::Zero(dim_);

    local_des_pos_ = Eigen::VectorXd::Zero(dim_);
    local_des_vel_ = Eigen::VectorXd::Zero(dim_);
    local_des_acc_ = Eigen::VectorXd::Zero(dim_);

    pos_ = Eigen::VectorXd::Zero(dim_);
    vel_ = Eigen::VectorXd::Zero(dim_);

    local_pos_ = Eigen::VectorXd::Zero(dim_);
    local_vel_ = Eigen::VectorXd::Zero(dim_);

    pos_err_ = Eigen::VectorXd::Zero(dim_);
    vel_err_ = Eigen::VectorXd::Zero(dim_);

    local_pos_err_ = Eigen::VectorXd::Zero(dim_);
    local_vel_err_ = Eigen::VectorXd::Zero(dim_);

    kp_ = Eigen::VectorXd::Zero(dim_);
    kd_ = Eigen::VectorXd::Zero(dim_);
    ki_ = Eigen::VectorXd::Zero(dim_);

    kp_ik_ = Eigen::VectorXd::Zero(dim_);

    op_cmd_ = Eigen::VectorXd::Zero(dim_);

    jacobian_ = Eigen::MatrixXd::Zero(dim_, robot_->NumQdot());
    jacobian_dot_q_dot_ = Eigen::VectorXd::Zero(dim_);
  }
  virtual ~Task() = default;

  // for orientation task, des_pos is a 4 dimensional vector [x,y,z,w]
  // for angular momentum task, des_pos is ignored
  void UpdateDesired(const Eigen::VectorXd &des_pos,
                     const Eigen::VectorXd &des_vel,
                     const Eigen::VectorXd &des_acc) {
    des_pos_ = des_pos;
    des_vel_ = des_vel;
    des_acc_ = des_acc;
  }

  virtual void UpdateOpCommand() = 0;
  virtual void UpdateOpCommand(const Eigen::Matrix3d &rot_world_local) = 0;
  virtual void UpdateJacobian() = 0;
  virtual void UpdateJacobianDotQdot() = 0;

  // setter function
  // task gain, hierarchy
  virtual void SetParameters(const YAML::Node &node, const bool b_sim) {
    try {
      std::string prefix = b_sim ? "sim" : "exp";
      util::ReadParameter(node, prefix + "_kp", kp_);
      util::ReadParameter(node, prefix + "_kd", kd_);
      util::ReadParameter(node, prefix + "_kp_ik", kp_ik_);
    } catch (std::runtime_error &e) {
      std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
                << __FILE__ << "]" << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }

  void ModifyJacobian(const std::vector<int> joint_idx) {
    for (int i(0); i < joint_idx.size(); i++)
      jacobian_.col(joint_idx[i]).setZero();
  }

  // getter function
  Eigen::VectorXd DesiredPos() const { return des_pos_; }
  Eigen::VectorXd DesiredVel() const { return des_vel_; }
  Eigen::VectorXd DesiredAcc() const { return des_acc_; }

  Eigen::VectorXd DesiredLocalPos() const { return local_des_pos_; }
  Eigen::VectorXd DesiredLocalVel() const { return local_des_vel_; }
  Eigen::VectorXd DesiredLocalAcc() const { return local_des_acc_; }

  Eigen::VectorXd CurrentPos() const { return pos_; }
  Eigen::VectorXd CurrentVel() const { return vel_; }

  Eigen::VectorXd CurrentLocalPos() const { return local_pos_; }
  Eigen::VectorXd CurrentLocalVel() const { return local_vel_; }

  Eigen::MatrixXd Jacobian() const { return jacobian_; }
  Eigen::MatrixXd JacobianDotQdot() const { return jacobian_dot_q_dot_; }
  Eigen::VectorXd Kp() const { return kp_; }
  Eigen::VectorXd Kd() const { return kd_; }
  Eigen::VectorXd Ki() const { return ki_; }
  Eigen::VectorXd KpIK() const { return kp_ik_; }
  Eigen::VectorXd OpCommand() const { return op_cmd_; }
  int Dim() const { return dim_; }

  Eigen::VectorXd PosError() const { return pos_err_; }
  Eigen::VectorXd LocalPosError() const { return local_pos_err_; }

  // TODO: virtual??
  int TargetIdx() const { return target_idx_; }
  Eigen::Matrix3d Rot() const { return rot_link_w_; }

  // setter
  void SetKp(Eigen::VectorXd kp) { kp_ = kp; }
  void SetKd(Eigen::VectorXd kd) { kd_ = kd; }
  void SetKi(Eigen::VectorXd ki) { ki_ = ki; }

  // Debug
  void Debug() {
    std::cout << "=================================" << std::endl;
    std::cout << "des_xddot: " << op_cmd_.transpose() << std::endl;
    std::cout << "pos_err: " << pos_err_.transpose() << std::endl;
    std::cout << "des_pos: " << des_pos_.transpose() << std::endl;
    std::cout << "pos: " << pos_.transpose() << std::endl;
    std::cout << "vel_err: " << vel_err_.transpose() << std::endl;
    std::cout << "des_vel: " << des_vel_.transpose() << std::endl;
    std::cout << "vel: " << vel_.transpose() << std::endl;
  }

protected:
  PinocchioRobotSystem *robot_;
  int dim_;
  int target_idx_;

  Eigen::Matrix3d rot_link_w_;

  // measured quantities
  Eigen::VectorXd pos_;
  Eigen::VectorXd vel_;

  Eigen::VectorXd local_pos_;
  Eigen::VectorXd local_vel_;

  Eigen::VectorXd pos_err_;
  Eigen::VectorXd vel_err_;

  Eigen::VectorXd local_pos_err_;
  Eigen::VectorXd local_vel_err_;

  // task space gains
  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;
  Eigen::VectorXd ki_;

  // ik gains
  Eigen::VectorXd kp_ik_;

  //  desired quantities
  Eigen::VectorXd des_pos_;
  Eigen::VectorXd des_vel_;
  Eigen::VectorXd des_acc_;

  Eigen::VectorXd local_des_pos_;
  Eigen::VectorXd local_des_vel_;
  Eigen::VectorXd local_des_acc_;

  Eigen::VectorXd op_cmd_;

  Eigen::MatrixXd jacobian_;
  Eigen::VectorXd jacobian_dot_q_dot_;
};

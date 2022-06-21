#pragma once
#include <Eigen/Dense>
#include <iostream>

#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "util/util.hpp"

class Task {
public:
  Task(PinocchioRobotSystem *robot, const int dim,
       const int *target_idx = nullptr)
      : robot_(robot), dim_(dim), target_idx_(target_idx) {

    des_pos_ = Eigen::VectorXd::Zero(dim_);
    des_vel_ = Eigen::VectorXd::Zero(dim_);
    des_acc_ = Eigen::VectorXd::Zero(dim_);

    pos_ = Eigen::VectorXd::Zero(dim_);
    vel_ = Eigen::VectorXd::Zero(dim_);

    pos_err_ = Eigen::VectorXd::Zero(dim_);
    vel_err_ = Eigen::VectorXd::Zero(dim_);

    kp_ = Eigen::VectorXd::Zero(dim_);
    kd_ = Eigen::VectorXd::Zero(dim_);
    ki_ = Eigen::VectorXd::Zero(dim_);

    osc_cmd_ = Eigen::VectorXd::Zero(dim_);

    jacobian_ = Eigen::MatrixXd::Zero(dim_, robot_->GetNumQdot());
    jacobian_dot_q_dot_ = Eigen::VectorXd::Zero(dim_);

    task_weight_ = Eigen::VectorXd::Zero(dim_);
  }
  virtual ~Task() = default;

  // for orientation task, des_pos is a 4 dimensional vector [x,y,z,w]
  // for angular momentum task, des_pos is ignored
  void UpdateDesiredTask(const Eigen::VectorXd &des_pos,
                         const Eigen::VectorXd &des_vel,
                         const Eigen::VectorXd &des_acc) {
    des_pos_ = des_pos;
    des_vel_ = des_vel;
    des_acc_ = des_acc;
  }

  virtual void UpdateOscCommand() = 0;
  virtual void UpdateTaskJacobian() = 0;
  virtual void UpdateTaskJacobianDotQdot() = 0;

  // setter function
  // task gain, hierarchy
  virtual void SetParameters(const YAML::Node &node, const bool b_sim) {
    try {
      kp_ = b_sim ? util::ReadParameter<Eigen::VectorXd>(node, "kp")
                  : util::ReadParameter<Eigen::VectorXd>(node, "exp_kp");
      kd_ = b_sim ? util::ReadParameter<Eigen::VectorXd>(node, "kd")
                  : util::ReadParameter<Eigen::VectorXd>(node, "exp_kd");
      task_weight_ =
          b_sim ? util::ReadParameter<Eigen::VectorXd>(node, "weight")
                : util::ReadParameter<Eigen::VectorXd>(node, "exp_weight");
    } catch (std::runtime_error &e) {
      std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
                << __FILE__ << "]" << std::endl
                << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }

  // getter function
  Eigen::VectorXd GetTaskDesiredPos() { return des_pos_; }
  Eigen::VectorXd GetTaskDesiredVel() { return des_vel_; }
  Eigen::VectorXd GetTaskDesiredAcc() { return des_acc_; }

  const int *GetTargetIdx() { return target_idx_; }

protected:
  PinocchioRobotSystem *robot_;
  const int dim_;
  const int *target_idx_;

  // measured quantities
  Eigen::VectorXd pos_;
  Eigen::VectorXd vel_;

  Eigen::VectorXd pos_err_;
  Eigen::VectorXd vel_err_;

  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;
  Eigen::VectorXd ki_;

  //  desired quantities
  Eigen::VectorXd des_pos_;
  Eigen::VectorXd des_vel_;
  Eigen::VectorXd des_acc_;

  Eigen::VectorXd osc_cmd_;

  Eigen::MatrixXd jacobian_;
  Eigen::VectorXd jacobian_dot_q_dot_;

  Eigen::VectorXd task_weight_;
};

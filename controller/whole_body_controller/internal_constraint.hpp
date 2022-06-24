#pragma once
#include <Eigen/Dense>

#include "controller/robot_system/pinocchio_robot_system.hpp"

class InternalConstraint {
public:
  InternalConstraint(PinocchioRobotSystem *robot, const int dim)
      : robot_(robot), dim_(dim) {
    jacobian_ = Eigen::MatrixXd::Zero(dim_, robot_->NumQdot());
    jacobian_dot_q_dot_ = Eigen::VectorXd::Zero(dim_);
  };
  virtual ~InternalConstraint() = default;

  virtual void UpdateJacobian() = 0;
  virtual void UpdateJacobianDotQdot() = 0;

  // getter
  Eigen::MatrixXd Jacobian() const { return jacobian_; }
  Eigen::VectorXd JacobianDotQdot() const { return jacobian_dot_q_dot_; }
  int Dim() const { return dim_; }

protected:
  PinocchioRobotSystem *robot_;
  int dim_;
  Eigen::MatrixXd jacobian_;
  Eigen::VectorXd jacobian_dot_q_dot_;
};

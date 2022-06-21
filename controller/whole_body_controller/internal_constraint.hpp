#pragma once
#include <Eigen/Dense>

class PinocchioRobotSystem;
class InternalConstraint {
public:
  InternalConstraint(PinocchioRobotSystem *robot, const int dim)
      : robot_(robot), dim_(dim) {
    jacobian_ = Eigen::MatrixXd::Zero(dim_, robot_->n_qdot_);
    jacobian_dot_q_dot_ = Eigen::VectorXd::Zero(dim_);
  };
  virtual ~InternalConstraint();

  virtual void UpdateJacobian() = 0;
  virtual void UpdateJacobianDotQdot() = 0;

  // getter
  const Eigen::MatrixXd GetJacobian() { return jacobian_; }
  const Eigen::VectorXd GetJacobianDotQdot() { return jacobian_dot_q_dot_; }

protected:
  PinocchioRobotSystem *robot_;
  int dim_;
  Eigen::MatrixXd jacobian_;
  Eigen::VectorXd jacobian_dot_q_dot_;
};

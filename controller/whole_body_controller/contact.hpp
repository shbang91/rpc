#pragma once
#include <Eigen/Dense>
#include <string>

#include "controller/robot_system/pinocchio_robot_system.hpp"

class Contact {
public:
  Contact(PinocchioRobotSystem *robot, const int &dim,
          const int &target_link_idx, const double &mu)
      : dim_(dim), mu_(mu), target_link_idx_(target_link_idx) {

    robot_ = robot;

    jacobian_ = Eigen::MatrixXd::Zero(dim_, robot_->GetNumQdot());
    jacobian_dot_q_dot_ = Eigen::VectorXd::Zero(dim_);

    rf_z_max_ = 1000.;
  };
  virtual ~Contact() = default;

  virtual void UpdateContactJacobian() = 0;
  virtual void UpdateContactJacobianDotQdot() = 0;
  virtual void UpdateConeConstraint() = 0;

protected:
  PinocchioRobotSystem *robot_;

  int dim_;
  int target_link_idx_;
  double mu_;

  Eigen::MatrixXd jacobian_;
  Eigen::VectorXd jacobian_dot_q_dot_;

  double rf_z_max_;
  Eigen::MatrixXd cone_constraint_matrix_;
  Eigen::VectorXd cone_constraint_vector_;
};

#pragma once
#include <Eigen/Dense>
#include <string>

#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "util/util.hpp"

class Contact {
public:
  Contact(PinocchioRobotSystem *robot, const int dim, const int target_link_idx,
          const double mu)
      : robot_(robot), dim_(dim), target_link_idx_(target_link_idx), mu_(mu),
        rf_z_max_(0.001) {

    jacobian_ = Eigen::MatrixXd::Zero(dim_, robot_->NumQdot());
    jacobian_dot_q_dot_ = Eigen::VectorXd::Zero(dim_);
  };
  virtual ~Contact() = default;

  virtual void UpdateJacobian() = 0;
  virtual void UpdateJacobianDotQdot() = 0;
  virtual void UpdateConeConstraint() = 0;

  // setter
  // contact mu, rf_z_max
  virtual void SetParameters(const YAML::Node &node, const bool b_sim) = 0;
  void SetMaxFz(const double rf_z_max) { rf_z_max_ = rf_z_max; }

  // getter
  double MaxFz() const { return rf_z_max_; }
  int Dim() const { return dim_; }

  Eigen::MatrixXd Jacobian() const { return jacobian_; }
  Eigen::VectorXd JacobianDotQdot() const { return jacobian_dot_q_dot_; }
  Eigen::MatrixXd UfMatrix() const { return cone_constraint_matrix_; }
  Eigen::VectorXd UfVector() const { return cone_constraint_vector_; }
  int TargetLinkIdx() const { return target_link_idx_; }

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

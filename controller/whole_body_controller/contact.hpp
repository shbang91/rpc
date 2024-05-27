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

  void SetDesiredPos(const Eigen::Vector3d &pos) { des_pos_ = pos; }
  void SetDesiredOri(const Eigen::Quaterniond &quat) { des_quat_ = quat; }

  // getter
  double MaxFz() const { return rf_z_max_; }
  int Dim() const { return dim_; }
  Eigen::MatrixXd R() const { return rot_w_l_; }

  Eigen::MatrixXd Jacobian() const { return jacobian_; }
  Eigen::VectorXd JacobianDotQdot() const { return jacobian_dot_q_dot_; }
  Eigen::MatrixXd UfMatrix() const { return cone_constraint_matrix_; }
  Eigen::VectorXd UfVector() const { return cone_constraint_vector_; }
  int TargetLinkIdx() const { return target_link_idx_; }
  Eigen::VectorXd OpCommand() {
    Eigen::VectorXd res = Eigen::VectorXd::Zero(6);
    // pos
    res.tail<3>() =
        Eigen::Vector3d::Constant(5000).cwiseProduct(
            des_pos_ -
            robot_->GetLinkIsometry(target_link_idx_).translation()) -
        Eigen::Vector3d::Constant(50).cwiseProduct(
            robot_->GetLinkSpatialVel(target_link_idx_).tail<3>());
    // ori
    Eigen::Quaterniond curr_quat =
        Eigen::Quaterniond(robot_->GetLinkIsometry(target_link_idx_).linear())
            .normalized();
    util::AvoidQuatJump(des_quat_, curr_quat);
    Eigen::Quaterniond quat_err = des_quat_ * curr_quat.inverse();
    Eigen::Vector3d so3 = Eigen::AngleAxisd(quat_err).axis();
    so3 *= Eigen::AngleAxisd(quat_err).angle();
    res.head<3>() = Eigen::Vector3d::Constant(5000).cwiseProduct(so3) -
                    Eigen::Vector3d::Constant(50).cwiseProduct(
                        robot_->GetLinkSpatialVel(target_link_idx_).head<3>());
    return res;
  }

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

  Eigen::MatrixXd rot_w_l_;

  Eigen::Vector3d des_pos_;
  Eigen::Quaterniond des_quat_;
};

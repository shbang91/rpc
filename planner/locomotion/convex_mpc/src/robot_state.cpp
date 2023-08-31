#include "convex_mpc/robot_state.hpp"

#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include <iostream>
#include <stdexcept>

namespace convexmpc {

RobotState::RobotState(const std::string &urdf,
                       const std::vector<std::string> &feet)
    : model_(), data_(), R_(Matrix3d::Identity()),
      quat_(Quaterniond::Identity()), pose_(Vector7d::Zero()),
      w_local_(Vector3d::Zero()), w_world_(Vector3d::Zero()), feet_(),
      fk_(4, Vector3d::Zero()), fk_skew_(4, Matrix3d::Zero()) {
  try {
    if (feet.size() != 4) {
      throw std::out_of_range("Invalid argument: feet.size() must be 4!");
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    std::exit(EXIT_FAILURE);
  }
  pinocchio::urdf::buildModel(urdf, pinocchio::JointModelFreeFlyer(), model_);
  data_ = pinocchio::Data(model_);
  for (const auto &e : feet) {
    feet_.push_back(model_.getFrameId(e));
  }
}

void RobotState::update(const Vector19d &q, const Vector18d &v) {
  pinocchio::framesForwardKinematics(model_, data_, q);
  pinocchio::centerOfMass(model_, data_, q, v, false);
  pinocchio::ccrba(model_, data_, q, v);
  quat_.coeffs() = q.template segment<4>(3);
  R_ = quat_.toRotationMatrix();
  pose_ = q.template head<7>();
  w_local_ = v.template segment<3>(3);
  w_world_ = R_ * w_local_;
  twist_local_ = v.template head<6>();
  twist_world_.template tail<3>().noalias() = R_ * v.template head<3>();
  twist_world_.template head<3>() = w_world_;
  for (int i = 0; i < 4; ++i) {
    fk_[i] = data_.oMf[feet_[i]].translation() - data_.com[0];
    pinocchio::skew(fk_[i], fk_skew_[i]);
  }
  for (pinocchio::JointIndex i(0);
       i < static_cast<pinocchio::JointIndex>(model_.njoints); ++i)
    mass_ += model_.inertias[i].mass();

  I_ = data_.Ig.matrix().block<3, 3>(3, 3);
}

} // namespace convexmpc

#include "convex_mpc/state_equation.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace convexmpc {

StateEquation::StateEquation(const double dt, const double m, const Matrix3d &I,
                             const Vector3d &g)
    : dt_(dt), m_(m), g_(g), R_(Matrix3d::Identity()), I_local_(I),
      I_global_(I), I_global_inv_(I_global_.inverse()),
      I_inv_r_skew_(4, Matrix3d::Zero()) {
  try {
    if (dt <= 0.0) {
      throw std::out_of_range("Invalid argument: dt must be positive!");
    }
    if (m <= 0.0) {
      throw std::out_of_range("Invalid argument: m must be positive!");
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    std::exit(EXIT_FAILURE);
  }
}

void StateEquation::initQP(QPData &qp_data) const {
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    qp_data.qp_[i].A.setZero();
    qp_data.qp_[i].A.setIdentity();
    qp_data.qp_[i].A *= dt_;
    qp_data.qp_[i].A.template block<3, 3>(3, 9) = dt_ * Matrix3d::Identity();
  }
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    qp_data.qp_[i].B.setZero();
    for (int j = 0; j < 4; ++j) {
      qp_data.qp_[i].B.template block<3, 3>(9, j * 3) =
          (dt_ / m_) * Matrix3d::Identity();
    }
  }
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    qp_data.qp_[i].b.setZero();
    qp_data.qp_[i].b.template tail<3>() = g_ * dt_;
  }
}

void StateEquation::setQP(const ContactSchedule &contact_schedule,
                          const RobotState &robot_state, QPData &qp_data) {
  // rotation matrix
  R_ = robot_state.R();
  // global inertia matrix
  I_global_inv_.noalias() = R_.transpose() * I_local_;
  I_global_.noalias() = I_global_inv_ * R_;
  I_global_inv_ = I_global_.inverse();
  // dynamics w.r.t. control input
  for (int i = 0; i < 4; ++i) {
    I_inv_r_skew_[i].noalias() =
        dt_ * I_global_inv_ * robot_state.getLegKinematicsSkew(i);
  }
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    qp_data.qp_[i].A.template block<3, 3>(0, 6) = dt_ * R_;
    int nu = 0;
    for (int j = 0; j < 4; ++j) {
      if (contact_schedule.isContactActive(contact_schedule.phase(i))[j]) {
        qp_data.qp_[i].B.template block<3, 3>(6, nu) = I_inv_r_skew_[j];
        qp_data.qp_[i].B.template block<3, 3>(9, nu) =
            (dt_ / m_) * Matrix3d::Identity();
        nu += 3;
      }
    }
  }
}

} // namespace convexmpc

#include "convex_mpc/state_equation.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "util/util.hpp"
#include <pinocchio/algorithm/frames.hpp>

StateEquation::StateEquation(const double dt, const double m,
                             const Matrix3d &I_body, const Vector3d &g)
    : dt_(dt), m_(m), g_(g), I_local_(I_body),
      // I_inv_r_skew_(4, Matrix3d::Zero()) {
      I_inv_r_skew_(2, Matrix3d::Zero()) {
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
    qp_data.qp_[i].A.setIdentity();
    qp_data.qp_[i].A.template block<3, 3>(3, 9) = dt_ * Matrix3d::Identity();
  }
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    qp_data.qp_[i].B.setZero();
    // point contact
    // for (int j = 0; j < 4; ++j) {
    // qp_data.qp_[i].B.template block<3, 3>(9, j * 3) =
    //(dt_ / m_) * Matrix3d::Identity();
    //}
    // surface contact
    for (int j = 0; j < 2; ++j) {
      qp_data.qp_[i].B.template block<3, 3>(9, j * 6) =
          (dt_ / m_) * Matrix3d::Identity();
    }
  }
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    qp_data.qp_[i].b.setZero();
    qp_data.qp_[i].b.template tail<3>() = g_ * dt_;
  }
}

/*
void StateEquation::setQP(const ContactSchedule &contact_schedule,
                          const RobotState &robot_state, QPData &qp_data) {
  // rotation matrix
  R_ = robot_state.R();
  // global inertia matrix
  I_global_inv_.noalias() = R_.transpose() * I_local_;
  I_global_.noalias() = I_global_inv_ * R_;
  I_global_inv_ = I_global_.inverse();
  // dynamics w.r.t. control input
  // for (int i = 0; i < 4; ++i) {
  for (int i = 0; i < 2; ++i) {
    I_inv_r_skew_[i].noalias() =
        dt_ * I_global_inv_ * robot_state.getLegKinematicsSkew(i);
  }
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    qp_data.qp_[i].A.template block<3, 3>(0, 6) = dt_ * R_;
    int nu = 0;
    // point contact (4 legs)
    // for (int j = 0; j < 4; ++j) {
    // if (contact_schedule.isContactActive(contact_schedule.phase(i))[j]) {
    // qp_data.qp_[i].B.template block<3, 3>(6, nu) = I_inv_r_skew_[j];
    // qp_data.qp_[i].B.template block<3, 3>(9, nu) =
    //(dt_ / m_) * Matrix3d::Identity();
    // nu += 3;
    //}
    // surface contact (2 legs)
    for (int j = 0; j < 2; ++j) {
      if (contact_schedule.isContactActive(contact_schedule.phase(i))[j]) {
        qp_data.qp_[i].B.template block<3, 3>(6, nu) = dt_ * I_global_inv_;
        qp_data.qp_[i].B.template block<3, 3>(9, nu) = Eigen::Matrix3d::Zero();
        nu += 3;
        qp_data.qp_[i].B.template block<3, 3>(6, nu) = I_inv_r_skew_[j];
        qp_data.qp_[i].B.template block<3, 3>(9, nu) =
            (dt_ / m_) * Eigen::Matrix3d::Identity();
        nu += 3;
      } else {
        // when the foot is in swing motions
        // TODO: rollout centroidal inertia
      }
    }
  }
}
*/

void StateEquation::setQP(const Vector12d &initial_state,
                          const std::vector<ContactState> &contact_trajectory,
                          const aligned_vector<Eigen::Vector3d> &feet_pos,
                          QPData &qp_data) {
  // rotation matrix
  Eigen::Matrix3d Ryaw = util::EulerZYXtoQuat(0.0, 0.0, initial_state[2])
                             .toRotationMatrix(); // yaw angle

  // TODO: rollout centroidal inertia
  // inertia matrix
  Eigen::Matrix3d I_world = Ryaw.transpose() * I_local_ * Ryaw;
  Eigen::Matrix3d I_world_inv = I_world.inverse();

  for (int i = 0; i < 2; ++i) {
    Eigen::Matrix3d r_skew;
    pinocchio::skew(feet_pos[i], r_skew);
    I_inv_r_skew_[i].noalias() = dt_ * I_world_inv * r_skew;
  }
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    qp_data.qp_[i].A.template block<3, 3>(0, 6) = dt_ * Ryaw;
    int nu = 0;
    // point contact (4 legs)
    // for (int j = 0; j < 4; ++j) {
    // if (contact_schedule.isContactActive(contact_schedule.phase(i))[j]) {
    // qp_data.qp_[i].B.template block<3, 3>(6, nu) = I_inv_r_skew_[j];
    // qp_data.qp_[i].B.template block<3, 3>(9, nu) =
    //(dt_ / m_) * Matrix3d::Identity();
    // nu += 3;
    //}
    // surface contact (2 legs)
    for (int j = 0; j < 2; ++j) {
      if (contact_trajectory[i].contact[j]) {
        // foot in contact
        qp_data.qp_[i].B.template block<3, 3>(6, nu) = dt_ * I_world_inv;
        qp_data.qp_[i].B.template block<3, 3>(9, nu) = Eigen::Matrix3d::Zero();
        nu += 3;
        qp_data.qp_[i].B.template block<3, 3>(6, nu) = I_inv_r_skew_[j];
        qp_data.qp_[i].B.template block<3, 3>(9, nu) =
            (dt_ / m_) * Eigen::Matrix3d::Identity();
        nu += 3;
      } else {
        // when the foot is in swing motions
      }
    }
  }
}
#include "convex_mpc/state_equation.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>

#include "util/util.hpp"

StateEquation::StateEquation(const double dt, const double m,
                             const Matrix3d &I_body, const Vector3d &g)
    : dt_(dt), m_(m), g_(g), I_local_(I_body),
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
    // A matrix
    qp_data.qp_[i].A.setIdentity();
    qp_data.qp_[i].A.template block<3, 3>(3, 9) = dt_ * Matrix3d::Identity();

    // B matrix
    qp_data.qp_[i].B.setZero();

    // b matrix
    qp_data.qp_[i].b.setZero();
    qp_data.qp_[i].b.template tail<3>() = g_ * dt_;
  }
}

void StateEquation::setQP(const Vector12d &initial_state,
                          const std::vector<ContactState> &contact_trajectory,
                          const aligned_vector<Vector12d> &des_state_trajectory,
                          const aligned_vector<Eigen::Vector3d> &feet_pos,
                          QPData &qp_data) {

  // TODO: check this!!! (orientation issue at 90 degrees)
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    // rotation matrix
    Eigen::Matrix3d Ryaw =
        util::EulerZYXtoQuat(0.0, 0.0, des_state_trajectory[i][2])
            .toRotationMatrix(); // yaw angle

    Eigen::Matrix3d I_world = Ryaw * I_local_ * Ryaw.transpose();
    Eigen::Matrix3d I_world_inv = I_world.inverse();

    for (int k = 0; k < 2; ++k) {
      Eigen::Matrix3d r_skew;
      pinocchio::skew(feet_pos[k], r_skew);
      I_inv_r_skew_[k].noalias() = I_world_inv * r_skew;
    }

    qp_data.qp_[i].A.template block<3, 3>(0, 6) = dt_ * Ryaw.transpose();
    // planer contact (2 legs)
    int nu = 0;
    for (int j = 0; j < 2; ++j) {
      if (contact_trajectory[i].contact[j]) {
        // foot in contact
        qp_data.qp_[i].B.template block<3, 3>(6, nu) = dt_ * I_world_inv;
        qp_data.qp_[i].B.template block<3, 3>(9, nu) = Eigen::Matrix3d::Zero();
        nu += 3;
        qp_data.qp_[i].B.template block<3, 3>(6, nu) = dt_ * I_inv_r_skew_[j];
        qp_data.qp_[i].B.template block<3, 3>(9, nu) =
            (dt_ / m_) * Eigen::Matrix3d::Identity();
        nu += 3;
      } else {
        // when the foot is in swing motions
        // do nothing
      }
    }
  }
}

void StateEquation::setQP(
    const Vector12d &initial_state,
    const std::vector<ContactState> &contact_trajectory,
    const aligned_vector<Vector12d> &des_state_trajectory,
    const aligned_vector<Eigen::Matrix3d> &des_inertia_traj,
    const aligned_vector<Eigen::Vector3d> &feet_pos, QPData &qp_data) {
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    // rotation matrix
    Eigen::Matrix3d Ryaw =
        // util::EulerZYXtoQuat(0.0, 0.0, des_state_trajectory[i][2])
        //.toRotationMatrix(); // yaw angle
        util::EulerZYXtoQuat(des_state_trajectory[i][0],
                             des_state_trajectory[i][1],
                             des_state_trajectory[i][2])
            .toRotationMatrix(); // roll, pitch, yaw angle

    // inertia matrix
    Eigen::Matrix3d I_world = Ryaw * des_inertia_traj[i] * Ryaw.transpose();
    Eigen::Matrix3d I_world_inv = I_world.inverse();

    for (int k = 0; k < 2; ++k) {
      Eigen::Matrix3d r_skew;
      pinocchio::skew(feet_pos[k], r_skew);
      I_inv_r_skew_[k].noalias() = I_world_inv * r_skew;
    }

    qp_data.qp_[i].A.template block<3, 3>(0, 6) = dt_ * Ryaw.transpose();

    // planer contact (2 legs)
    int nu = 0;
    for (int j = 0; j < 2; ++j) {
      if (contact_trajectory[i].contact[j]) {
        // foot in contact
        qp_data.qp_[i].B.template block<3, 3>(6, nu) = dt_ * I_world_inv;
        qp_data.qp_[i].B.template block<3, 3>(9, nu) = Eigen::Matrix3d::Zero();
        nu += 3;
        qp_data.qp_[i].B.template block<3, 3>(6, nu) = dt_ * I_inv_r_skew_[j];
        qp_data.qp_[i].B.template block<3, 3>(9, nu) =
            (dt_ / m_) * Eigen::Matrix3d::Identity();
        nu += 3;
      } else {
        // when the foot is in swing motions
      }
    }
  }
}
void StateEquation::setQP(
    const Vector12d &initial_state,
    const std::vector<ContactState> &contact_trajectory,
    const aligned_vector<Vector12d> &des_state_trajectory,
    const aligned_vector<Eigen::Matrix3d> &des_inertia_traj,
    const std::vector<aligned_vector<Eigen::Vector3d>> &feet_pos_traj,
    QPData &qp_data) {
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    // rotation matrix
    Eigen::Matrix3d Ryaw =
        util::EulerZYXtoQuat(0.0, 0.0, des_state_trajectory[i][2])
            .toRotationMatrix(); // yaw angle
    // util::EulerZYXtoQuat(des_state_trajectory[i][0],
    // des_state_trajectory[i][1],
    // des_state_trajectory[i][2])
    //.toRotationMatrix(); // roll, pitch, yaw angle

    // inertia matrix
    Eigen::Matrix3d I_world = Ryaw * des_inertia_traj[i] * Ryaw.transpose();
    Eigen::Matrix3d I_world_inv = I_world.inverse();

    for (int k = 0; k < 2; ++k) {
      Eigen::Matrix3d r_skew;
      pinocchio::skew(feet_pos_traj[k][i], r_skew);
      I_inv_r_skew_[k].noalias() = dt_ * I_world_inv * r_skew;
    }

    qp_data.qp_[i].A.template block<3, 3>(0, 6) = Ryaw.transpose();

    // planer contact (2 legs)
    int nu = 0;
    for (int j = 0; j < 2; ++j) {
      if (contact_trajectory[i].contact[j]) {
        // foot in contact
        qp_data.qp_[i].B.template block<3, 3>(6, nu) = dt_ * I_world_inv;
        qp_data.qp_[i].B.template block<3, 3>(9, nu) = Eigen::Matrix3d::Zero();
        nu += 3;
        qp_data.qp_[i].B.template block<3, 3>(6, nu) = dt_ * I_inv_r_skew_[j];
        qp_data.qp_[i].B.template block<3, 3>(9, nu) =
            (dt_ / m_) * Eigen::Matrix3d::Identity();
        nu += 3;
      } else {
        // when the foot is in swing motions
      }
    }
  }
}

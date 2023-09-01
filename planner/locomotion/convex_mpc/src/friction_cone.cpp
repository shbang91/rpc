#include "convex_mpc/friction_cone.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace convexmpc {

// FrictionCone::FrictionCone(const double mu, const double fzmin,
// const double fzmax)
//: mu_(mu), fzmin_(fzmin), fzmax_(fzmax), cone_(MatrixXd::Zero(16, 12)) {
// try {
// if (mu <= 0.0) {
// throw std::out_of_range("Invalid argument: mu must be positive!");
//}
// if (fzmin < 0.0) {
// throw std::out_of_range("Invalid argument: fzmin must be non-negative!");
//}
// if (fzmax <= fzmin) {
// throw std::out_of_range(
//"Invalid argument: fzmax must be larger than fzmin!");
//}
//} catch (const std::exception &e) {
// std::cerr << e.what() << '\n';
// std::exit(EXIT_FAILURE);
//}
// MatrixXd cone(4, 3);
// cone << 1.0, 0.0, -(mu / std::sqrt(2)), -1.0, 0.0, -(mu / std::sqrt(2)), 0.0,
// 1.0, -(mu / std::sqrt(2)), 0.0, -1.0, -(mu / std::sqrt(2));
// for (int i = 0; i < 4; ++i) {
// cone_.block(4 * i, 3 * i, 4, 3) = cone;
//}
//}

FrictionCone::FrictionCone(const double mu, const double fzmin,
                           const double fzmax, const double x, const double y)
    : mu_(mu), fzmin_(fzmin), fzmax_(fzmax), x_(x), y_(y),
      cone_(MatrixXd::Zero(16 * 2, 12)) {
  try {
    if (mu <= 0.0) {
      throw std::out_of_range("Invalid argument: mu must be positive!");
    }
    if (fzmin < 0.0) {
      throw std::out_of_range("Invalid argument: fzmin must be non-negative!");
    }
    if (fzmax <= fzmin) {
      throw std::out_of_range(
          "Invalid argument: fzmax must be larger than fzmin!");
    }
    if (x < 0.0) {
      throw std::out_of_range("Invalid argument: x must be non-negative!");
    }
    if (y < 0.0) {
      throw std::out_of_range("Invalid argument: y must be non-negative!");
    }
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    std::exit(EXIT_FAILURE);
  }
  MatrixXd cone(16, 6);
  cone.setZero();
  // Coulomb friction cone constraint
  cone(0, 3) = 1.;
  cone(0, 5) = mu_;
  cone(1, 3) = -1.;
  cone(1, 5) = mu_;

  cone(2, 4) = 1.;
  cone(2, 5) = mu_;
  cone(3, 4) = -1.;
  cone(3, 5) = mu_;

  // Cop condition
  cone(4, 0) = 1.;
  cone(4, 5) = y_;
  cone(5, 0) = -1.;
  cone(5, 5) = y_;

  cone(6, 1) = 1.;
  cone(6, 5) = x_;
  cone(7, 1) = -1.;
  cone(7, 5) = x_;

  // no-yaw-slippage condition
  cone(8, 0) = -mu_;
  cone(8, 1) = -mu_;
  cone(8, 2) = 1.;
  cone(8, 3) = y_;
  cone(8, 4) = x_;
  cone(8, 5) = (x_ + y_) * mu_;

  cone(9, 0) = -mu_;
  cone(9, 1) = mu_;
  cone(9, 2) = 1.;
  cone(9, 3) = y_;
  cone(9, 4) = -x_;
  cone(9, 5) = (x_ + y_) * mu_;

  cone(10, 0) = mu_;
  cone(10, 1) = -mu_;
  cone(10, 2) = 1.;
  cone(10, 3) = -y_;
  cone(10, 4) = x_;
  cone(10, 5) = (x_ + y_) * mu_;

  cone(11, 0) = mu_;
  cone(11, 1) = mu_;
  cone(11, 2) = 1.;
  cone(11, 3) = -y_;
  cone(11, 4) = -x_;
  cone(11, 5) = (x_ + y_) * mu_;
  /////////////////////////////////////////////////
  cone(12, 0) = -mu_;
  cone(12, 1) = -mu_;
  cone(12, 2) = -1.;
  cone(12, 3) = -y_;
  cone(12, 4) = -x_;
  cone(12, 5) = (x_ + y_) * mu_;

  cone(13, 0) = -mu_;
  cone(13, 1) = mu_;
  cone(13, 2) = -1.;
  cone(13, 3) = -y_;
  cone(13, 4) = x_;
  cone(13, 5) = (x_ + y_) * mu_;

  cone(14, 0) = mu_;
  cone(14, 1) = -mu_;
  cone(14, 2) = -1.;
  cone(14, 3) = y_;
  cone(14, 4) = -x_;
  cone(14, 5) = (x_ + y_) * mu_;

  cone(15, 0) = mu_;
  cone(15, 1) = mu_;
  cone(15, 2) = -1.;
  cone(15, 3) = y_;
  cone(15, 4) = x_;
  cone(15, 5) = (x_ + y_) * mu_;

  for (int i = 0; i < 2; ++i) {
    cone_.block(16 * i, 6 * i, 16, 6) = cone;
  }
}
FrictionCone::FrictionCone() : mu_(), fzmin_(), fzmax_() {}

void FrictionCone::setQP(QPData &qp_data) const {
  // box constraints
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    for (int j = 0; j < qp_data.dim_.nbu[i]; ++j) {
      // qp_data.qp_[i].idxbu[j] = 3 * j + 2;
      qp_data.qp_[i].idxbu[j] = 6 * j + 5;
    }
    // qp_data.qp_[i].lbu.fill(fzmin_);
    // qp_data.qp_[i].ubu.fill(fzmax_);
    qp_data.qp_[i].lbu =
        Eigen::VectorXd::Constant(qp_data.qp_[i].lbu.size(), fzmin_);
    qp_data.qp_[i].ubu =
        Eigen::VectorXd::Constant(qp_data.qp_[i].ubu.size(), fzmax_);
    // if (std::isinf(fzmax_)) {
    // qp_data.qp_[i].ubu_mask.fill(1.0);
    // qp_data.qp_[i].ubu_mask =
    // Eigen::VectorXd::Constant(qp_data.qp_[i].ubu_mask.size(), 0.0);
    //}
  }

  // inequality constraints
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    qp_data.qp_[i].C.setZero();
    const int num_conatcts = qp_data.dim_.nbu[i];
    // assert(qp_data.dim_.nbu[i] % 3 == 0);
    if (num_conatcts > 0) {
      qp_data.qp_[i].D =
          cone_.topLeftCorner(qp_data.qp_[i].D.rows(), qp_data.qp_[i].D.cols());
    }
    qp_data.qp_[i].lg.setZero();
    qp_data.qp_[i].ug.setZero();
    qp_data.qp_[i].lg_mask = Eigen::VectorXd::Constant(
        // qp_data.qp_[i].lg_mask.size(), 0.0); // disable lower bounds
        qp_data.qp_[i].ug_mask.size(), 0.0); // disable upper bounds
  }
}

} // namespace convexmpc

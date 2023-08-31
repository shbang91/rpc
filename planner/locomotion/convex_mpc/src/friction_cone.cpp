#include "convex_mpc/friction_cone.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace convexmpc {

FrictionCone::FrictionCone(const double mu, const double fzmin,
                           const double fzmax)
    : mu_(mu), fzmin_(fzmin), fzmax_(fzmax), cone_(MatrixXd::Zero(16, 12)) {
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
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    std::exit(EXIT_FAILURE);
  }
  MatrixXd cone(4, 3);
  cone << 1.0, 0.0, -(mu / std::sqrt(2)), -1.0, 0.0, -(mu / std::sqrt(2)), 0.0,
      1.0, -(mu / std::sqrt(2)), 0.0, -1.0, -(mu / std::sqrt(2));
  for (int i = 0; i < 4; ++i) {
    cone_.block(4 * i, 3 * i, 4, 3) = cone;
  }
}

FrictionCone::FrictionCone() : mu_(), fzmin_(), fzmax_() {}

void FrictionCone::setQP(QPData &qp_data) const {
  // box constraints
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    for (int j = 0; j < qp_data.dim_.nbu[i]; ++j) {
      qp_data.qp_[i].idxbu[j] = 3 * j + 2;
    }
    // qp_data.qp_[i].lbu.fill(fzmin_);
    // qp_data.qp_[i].ubu.fill(fzmax_);
    qp_data.qp_[i].lbu =
        Eigen::VectorXd::Constant(qp_data.qp_[i].lbu.size(), fzmin_);
    qp_data.qp_[i].ubu =
        Eigen::VectorXd::Constant(qp_data.qp_[i].ubu.size(), fzmax_);
    if (std::isinf(fzmax_)) {
      // qp_data.qp_[i].ubu_mask.fill(1.0);
      qp_data.qp_[i].ubu_mask =
          Eigen::VectorXd::Constant(qp_data.qp_[i].ubu_mask.size(), 0.0);
    }
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
        qp_data.qp_[i].lg_mask.size(), 0.0); // disable lower bounds
  }
}

} // namespace convexmpc

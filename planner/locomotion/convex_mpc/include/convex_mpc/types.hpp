#ifndef CONVEX_MPC_TYPES_HPP_
#define CONVEX_MPC_TYPES_HPP_

#include <vector>

#include "Eigen/Core"
#include "Eigen/LU"
#include "Eigen/StdVector"

#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/spatial/se3.hpp"

using MatrixXd = Eigen::MatrixXd;
using VectorXd = Eigen::VectorXd;

using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector4d = Eigen::Matrix<double, 3, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using Vector7d = Eigen::Matrix<double, 7, 1>;
using Vector9d = Eigen::Matrix<double, 9, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;
using Vector19d = Eigen::Matrix<double, 19, 1>;
using Vector18d = Eigen::Matrix<double, 18, 1>;
using Vector34d = Eigen::Matrix<double, 34, 1>;
using Vector33d = Eigen::Matrix<double, 33, 1>;

using Quaterniond = Eigen::Quaternion<double>;
using Eigen::Block;
using Eigen::VectorBlock;

template <typename T>
using aligned_vector = std::vector<T, Eigen::aligned_allocator<T>>;

using pinocchio::SE3;

struct ContactState {
  union {
    bool contact[2];
    struct {
      bool lf, rf;
    };
  };

  ContactState(bool _lf, bool _rf) {
    lf = _lf;
    rf = _rf;
  }

  ContactState() {}
};

#endif // CONVEX_MPC_TYPES_HPP_

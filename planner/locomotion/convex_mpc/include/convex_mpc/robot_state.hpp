#ifndef CONVEX_MPC_ROBOT_STATE_HPP_
#define CONVEX_MPC_ROBOT_STATE_HPP_

#include <cassert>
#include <string>
#include <vector>

#include "pinocchio/multibody/data.hpp"
#include "pinocchio/multibody/model.hpp"

#include "convex_mpc/types.hpp"

namespace convexmpc {

class RobotState {
public:
  RobotState(const std::string &urdf, const std::vector<std::string> &feet);

  RobotState() = default;

  ~RobotState() = default;

  void update(const Vector19d &q, const Vector18d &v);

  const double &mass() const { return mass_; }

  const Eigen::Matrix3d &I() const { return I_; }

  const Vector3d &com() const { return data_.com[0]; }

  const Matrix3d &R() const { return R_; }

  const Quaterniond &quat() const { return quat_; }

  const Vector7d pose() const { return pose_; }

  const Vector3d &vcom() const { return data_.vcom[0]; }

  const Vector3d &wcom() const { return w_world_; }

  const Vector6d &twist() const { return twist_world_; }

  const Vector3d &getLegKinematics(const int i) const {
    assert(i >= 0);
    assert(i < 4);
    return fk_[i];
  }

  const Matrix3d &getLegKinematicsSkew(const int i) const {
    assert(i >= 0);
    assert(i < 4);
    return fk_skew_[i];
  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
  Quaterniond quat_;
  Matrix3d R_;
  Vector7d pose_;
  Vector3d w_local_, w_world_;
  Vector6d twist_local_, twist_world_;
  std::vector<int> feet_;
  aligned_vector<Vector3d> fk_;
  aligned_vector<Matrix3d> fk_skew_;

  double mass_;
  Eigen::Matrix3d I_;
};

} // namespace convexmpc

#endif // CONVEX_MPC_ROBOT_STATE_HPP_

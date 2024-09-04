#ifndef CONVEX_MPC_MPC_SOLUTION_HPP_
#define CONVEX_MPC_MPC_SOLUTION_HPP_

#include "convex_mpc/contact_schedule.hpp"
#include "convex_mpc/qp_data.hpp"
// #include "convex_mpc/robot_state.hpp"
// #include "convex_mpc/single_rigid_body.hpp"
#include "convex_mpc/types.hpp"

#include "util/util.hpp"

class MPCSolution {
public:
  MPCSolution() = default;

  ~MPCSolution() = default;

  void init(const int horizon_length, const double mpc_dt);

  void update(const std::vector<ContactState> &contact_trajectory,
              const QPData &qp_data);

  const std::vector<double> &time() const { return time_; }

  const aligned_vector<Vector3d> &pos() const { return pos_; }

  const aligned_vector<Quaterniond> &quat() const { return quat_; }

  const aligned_vector<Vector3d> &euler() const { return euler_xyz_; }

  const aligned_vector<Matrix3d> &R() const { return R_; }

  const aligned_vector<Vector7d> &pose() const { return pose_; }

  const aligned_vector<Vector6d> &twist() const { return twist_; }

  const aligned_vector<Vector3d> &v() const { return v_; }

  const aligned_vector<Vector3d> &w() const { return w_; }

  // const aligned_vector<aligned_vector<Vector3d>> &f() const { return f_; }
  const aligned_vector<aligned_vector<Vector6d>> &f() const { return f_; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  int N_;
  double mpc_dt_;
  std::vector<double> time_;
  aligned_vector<Vector3d> pos_;
  aligned_vector<Quaterniond> quat_;
  aligned_vector<Vector3d> euler_xyz_;
  aligned_vector<Matrix3d> R_;
  aligned_vector<Vector7d> pose_;
  aligned_vector<Vector6d> twist_;
  aligned_vector<Vector3d> v_, w_;
  // aligned_vector<aligned_vector<Vector3d>> f_;
  aligned_vector<aligned_vector<Vector6d>> f_;
  // SingleRigidBody single_rigid_body_;
};

#endif // CONVEX_MPC_MPC_SOLUTION_HPP_

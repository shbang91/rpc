#ifndef CONVEX_MPC_STATE_EQUATION_HPP_
#define CONVEX_MPC_STATE_EQUATION_HPP_

#include <vector>

#include "convex_mpc/contact_schedule.hpp"
#include "convex_mpc/qp_data.hpp"
#include "convex_mpc/robot_state.hpp"
#include "convex_mpc/types.hpp"

namespace convexmpc {

class StateEquation {
public:
  StateEquation(const double dt, const double m = 22.5,
                const Matrix3d &I = (Matrix3d() << 0.050874, 0., 0., 0.,
                                     0.64036, 0., 0., 0., 0.6565)
                                        .finished(),
                const Vector3d &g = (Vector3d() << 0., 0., -9.81).finished());

  StateEquation() = default;

  ~StateEquation() = default;

  void initQP(QPData &qp_data) const;

  void setQP(const Vector12d &initial_state,
             const std::vector<ContactState> &contact_trajectory,
             const Vector6d &feet_pos, QPData &qp_data);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  int N_;
  double dt_, m_;
  Vector3d g_;
  Matrix3d I_local_;
  aligned_vector<Matrix3d> I_inv_r_skew_;
};

} // namespace convexmpc

#endif // CONVEX_MPC_STATE_EQUATION_HPP_

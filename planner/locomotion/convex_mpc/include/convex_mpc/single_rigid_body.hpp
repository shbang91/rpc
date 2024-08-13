#ifndef CONVEX_MPC_SINGLE_RIGID_BODY_HPP_
#define CONVEX_MPC_SINGLE_RIGID_BODY_HPP_

#include <vector>

#include "convex_mpc/contact_schedule.hpp"
#include "convex_mpc/qp_data.hpp"
#include "convex_mpc/robot_state.hpp"
#include "convex_mpc/types.hpp"

class SingleRigidBody {
public:
  SingleRigidBody();

  ~SingleRigidBody() = default;

  void integrate(const Vector6d &dq, Vector7d &q) const;

  void integrate(const Vector7d &q, const Vector6d &dq, Vector7d &q_next) const;

  void difference(const Vector7d &qf, const Vector7d &q0,
                  Vector6d &qdiff) const;

  void dDifference_dqf(const Vector7d &qf, const Vector7d &q0,
                       Matrix6d &Jdiff) const;

  void dDifference_dq0(const Vector7d &qf, const Vector7d &q0,
                       Matrix6d &Jdiff) const;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  pinocchio::Model model_;
  pinocchio::Data data_;
};

#endif // CONVEX_MPC_SINGLE_RIGID_BODY_HPP_

#include "convex_mpc/single_rigid_body.hpp"

#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

namespace convexmpc {

SingleRigidBody::SingleRigidBody() : model_(), data_() {
  const auto ffidx = model_.addJoint(0, pinocchio::JointModelFreeFlyer(),
                                     SE3::Identity(), "floating_base_joint");
  model_.addJointFrame(ffidx);
  model_.appendBodyToJoint(ffidx, pinocchio::Inertia::Zero(), SE3::Identity());
  model_.addBodyFrame("floating_base_body", ffidx);
  model_.addBodyFrame("base_body", ffidx);
}

void SingleRigidBody::integrate(const Vector6d &dq, Vector7d &q) const {
  const Vector7d q_tmp = q;
  pinocchio::integrate(model_, q_tmp, dq, q);
}

void SingleRigidBody::integrate(const Vector7d &q, const Vector6d &dq,
                                Vector7d &q_next) const {
  pinocchio::integrate(model_, q, dq, q_next);
}

void SingleRigidBody::difference(const Vector7d &qf, const Vector7d &q0,
                                 Vector6d &qdiff) const {
  pinocchio::difference(model_, q0, qf, qdiff);
}

void SingleRigidBody::dDifference_dqf(const Vector7d &qf, const Vector7d &q0,
                                      Matrix6d &Jdiff) const {
  pinocchio::dDifference(model_, q0, qf, Jdiff, pinocchio::ARG1);
}

void SingleRigidBody::dDifference_dq0(const Vector7d &qf, const Vector7d &q0,
                                      Matrix6d &Jdiff) const {
  pinocchio::dDifference(model_, q0, qf, Jdiff, pinocchio::ARG0);
}

} // namespace convexmpc

#include "controller/draco_controller/draco_rolling_joint_constraint.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "util/util.hpp"

DracoRollingJointConstraint::DracoRollingJointConstraint(
    PinocchioRobotSystem *robot)
    : InternalConstraint(robot, 2) {

  util::PrettyConstructor(3, "DracoRollingJointConstraint");
}

void DracoRollingJointConstraint::UpdateJacobian() {
  int l_knee_fe_jp = robot_->GetQdotIdx(draco_joint::l_knee_fe_jp);
  int l_knee_fe_jd = robot_->GetQdotIdx(draco_joint::l_knee_fe_jd);
  int r_knee_fe_jp = robot_->GetQdotIdx(draco_joint::r_knee_fe_jp);
  int r_knee_fe_jd = robot_->GetQdotIdx(draco_joint::r_knee_fe_jd);

  jacobian_(0, l_knee_fe_jp) = -1.;
  jacobian_(0, l_knee_fe_jd) = 1.;
  jacobian_(1, r_knee_fe_jp) = -1.;
  jacobian_(1, r_knee_fe_jd) = 1.;
}

void DracoRollingJointConstraint::UpdateJacobianDotQdot() {
  jacobian_dot_q_dot_.setZero();
}

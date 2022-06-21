#pragma once
#include "controller/whole_body_controller/internal_constraint.hpp"

class DracoRollingJointConstraint : public InternalConstraint {
public:
  DracoRollingJointConstraint(PinocchioRobotSystem *robot);
  virtual ~DracoRollingJointConstraint() = default;

  void UpdateInternalConstraintJacobian() override;
  void UpdateInternalConstraintJacobianDotQdot() override;
};

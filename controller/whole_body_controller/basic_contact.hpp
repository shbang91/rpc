#pragma once
#include "controller/whole_body_controller/contact.hpp"

class PointContact : public Contact {
public:
  PointContact(PinocchioRobotSystem *robot, const int target_link_idx,
               const double mu);
  virtual ~PointContact() = default;

  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;
  void UpdateConeConstraint() override;
  void UpdateOpCommand() override;
  void SetParameters(const YAML::Node &node) override;
};

class SurfaceContact : public Contact {
public:
  SurfaceContact(PinocchioRobotSystem *robot, const int target_link_idx,
                 const double mu, const double x, const double y);
  virtual ~SurfaceContact() = default;

  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;
  void UpdateConeConstraint() override;
  void UpdateOpCommand() override;
  void SetParameters(const YAML::Node &node) override;

private:
  double x_;
  double y_;
};

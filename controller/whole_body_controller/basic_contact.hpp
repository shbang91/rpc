#pragma once
#include "controller/whole_body_controller/contact.hpp"

class PointContact : public Contact {
public:
  PointContact(PinocchioRobotSystem *robot, const int target_link_idx,
               const double mu);
  virtual ~PointContact() = default;

  void UpdateContactJacobian() override;
  void UpdateContactJacobianDotQdot() override;
  void UpdateConeConstraint() override;
  void SetParameters(const YAML::Node &node, const bool b_sim) override;
};

class SurfaceContact : public Contact {
public:
  SurfaceContact(PinocchioRobotSystem *robot, const int target_link_idx,
                 const double mu, const double x, const double y);
  virtual ~SurfaceContact() = default;

  void UpdateContactJacobian() override;
  void UpdateContactJacobianDotQdot() override;
  void UpdateConeConstraint() override;
  void SetParameters(const YAML::Node &node, const bool b_sim) override;

private:
  double x_;
  double y_;
};

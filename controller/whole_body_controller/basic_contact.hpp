#pragma once
#include "controller/whole_body_controller/contact.hpp"

class PointContact : public Contact {
public:
  PointContact(PinocchioRobotSystem *robot, const int &target_link_idx,
               const double &_mu);
  virtual ~PointContact() = default;

  void UpdateContactJacobian();
  void UpdateContactJacobianDotQdot();
  void UpdateConeConstraint();
};

class SurfaceContact : public Contact {
public:
  SurfaceContact(PinocchioRobotSystem *robot, const int &target_link_idx,
                 const double &mu, const double &x, const double &y);
  virtual ~SurfaceContact() = default;

  void UpdateContactJacobian();
  void UpdateContactJacobianDotQdot();
  void UpdateConeConstraint();

private:
  double x_;
  double y_;
};

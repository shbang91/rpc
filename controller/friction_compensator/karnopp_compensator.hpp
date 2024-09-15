#pragma once

/***
 * Friction compensator based on a modified version of the Karnopp friction
 * model. This version assumes a quadratic friction force instead of the
 * zero-force dead-zone in the Karnopp model. Moreover, it assumes a linear
 * viscous friction model after reaching the coulomb friction at the
 * dead-zone velocity. This force is meant to be used as a feedforward
 * torque.
 *
 * Created on: 8/17/2023
 */

#include <boost/math/special_functions/sign.hpp>

class KarnoppCompensator {
public:
  KarnoppCompensator(double static_force, double viscous_force,
                     double vel_deadzone);
  ~KarnoppCompensator();

  double Update(double des_vel);

protected:
  void ConstructModel();

  // force parameters defining the Stribeck curve
  double force_static_;
  double force_coulomb_;
  double viscous_friction_; // slope of linear viscous friction model (positive)

  // parameters for modified Karnopp model
  double vel_deadzone_;
  double alpha_Dv_; // coefficient for 2nd order model used in dead-zone region
  double bias_viscous_; // bias in linear viscous model mx + b
};

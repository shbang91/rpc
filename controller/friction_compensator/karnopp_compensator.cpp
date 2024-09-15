#include "karnopp_compensator.hpp"

using namespace boost::math;

KarnoppCompensator::KarnoppCompensator(double static_force,
                                       double viscous_force,
                                       double vel_deadzone) {
  force_static_ = static_force;
  force_coulomb_ = static_force;
  viscous_friction_ = viscous_force;

  vel_deadzone_ = vel_deadzone;

  ConstructModel();
}

KarnoppCompensator::~KarnoppCompensator() {}

double KarnoppCompensator::Update(double des_vel) {

  // check if in 'dead-zone' region
  if (std::abs(des_vel) < vel_deadzone_) {
    return sign(des_vel) * alpha_Dv_ * des_vel * des_vel;
  }

  return viscous_friction_ * des_vel + sign(des_vel) * bias_viscous_;
}

void KarnoppCompensator::ConstructModel() {

  //
  // dead-zone parameters
  //
  alpha_Dv_ =
      force_static_ / (vel_deadzone_ * vel_deadzone_); // F_s = alpha * v^2

  //
  // linear viscous force model parameters
  //
  bias_viscous_ = force_coulomb_ - viscous_friction_ * vel_deadzone_;
}

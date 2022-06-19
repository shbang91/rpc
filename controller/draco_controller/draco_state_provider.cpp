#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "util/util.hpp"

DracoStateProvider *DracoStateProvider::GetStateProvider() {
  static DracoStateProvider state_provider;
  return &state_provider;
}

DracoStateProvider::DracoStateProvider() {
  util::PrettyConstructor(1, "DracoStateProvider");
  current_time_ = 0.;

  stance_foot_ = draco_link::l_foot_contact;
  prev_stance_foot_ = draco_link::l_foot_contact;

  dcm_.setZero();
  prev_dcm_.setZero();
  dcm_vel_.setZero();

  b_lf_contact_ = true;
  b_rf_contact_ = true;

  state_ = draco_states::kInitialize;
  prev_state_ = draco_states::kInitialize;
}

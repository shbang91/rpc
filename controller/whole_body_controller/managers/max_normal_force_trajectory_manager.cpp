#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_contact.hpp"

MaxNormalForceTrajectoryManager::MaxNormalForceTrajectoryManager(
    Contact *contact, const double max_rf_z)
    : contact_(contact), max_rf_z_(max_rf_z) {
  util::PrettyConstructor(2, "MaxNormalForceTrajectoryManager");

  min_rf_z_ = 0.001;
  init_max_rf_z_ = 0.;   // default
  fin_max_rf_z_ = 0.;    // default
  interp_duration_ = 0.; // default
}

void MaxNormalForceTrajectoryManager::InitializeRampToMax(
    const double interp_duration) {
  interp_duration_ = interp_duration;
  fin_max_rf_z_ = max_rf_z_;
  init_max_rf_z_ = contact_->MaxFz();
}

void MaxNormalForceTrajectoryManager::InitializeRampToMin(
    const double interp_duration) {
  interp_duration_ = interp_duration;
  fin_max_rf_z_ = min_rf_z_;
  init_max_rf_z_ = contact_->MaxFz();
}

void MaxNormalForceTrajectoryManager::UpdateRampToMax(
    const double state_machine_time) {
  double rf_z_max = (state_machine_time <= interp_duration_)
                        ? init_max_rf_z_ + (fin_max_rf_z_ - init_max_rf_z_) /
                                               interp_duration_ *
                                               state_machine_time
                        : fin_max_rf_z_;

  contact_->SetMaxFz(rf_z_max);
}

void MaxNormalForceTrajectoryManager::UpdateRampToMin(
    const double state_machine_time) {
  double rf_z_max = (state_machine_time <= interp_duration_)
                        ? init_max_rf_z_ + (fin_max_rf_z_ - init_max_rf_z_) /
                                               interp_duration_ *
                                               state_machine_time
                        : fin_max_rf_z_;

  contact_->SetMaxFz(rf_z_max);
}

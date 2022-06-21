#pragma once

class Contact;
class MaxNormalForceTrajectoryManager {
public:
  MaxNormalForceTrajectoryManager(Contact *contact, const double max_rf_z_max);
  virtual ~MaxNormalForceTrajectoryManager() = default;

  void InitializeRampToMax(const double interp_duration);
  void InitializeRampToMin(const double interp_duration);

  void UpdateRampToMax(const double state_machine_time);
  void UpdateRampToMin(const double state_machine_time);

private:
  Contact *contact_;

  double max_rf_z_;
  double min_rf_z_;

  double init_max_rf_z_;
  double fin_max_rf_z_;
  double interp_duration_;
};

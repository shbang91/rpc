#pragma once
#include <Eigen/Dense>

class DracoStateProvider {
public:
  static DracoStateProvider *GetStateProvider();
  ~DracoStateProvider() = default;

  // servo dt should be set outside of controller
  double servo_dt_;

  double current_time_;

  int stance_foot_;
  int prev_stance_foot_;

  Eigen::Vector3d dcm_;
  Eigen::Vector3d prev_dcm_;
  Eigen::Vector3d dcm_vel_;

  bool b_lf_contact_;
  bool b_rf_contact_;

  int state_;
  int prev_state_;

  // should be set outside of controller
  Eigen::VectorXd nominal_jpos_;

private:
  DracoStateProvider();
};

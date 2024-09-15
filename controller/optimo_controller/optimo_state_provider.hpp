#pragma once
#include <Eigen/Dense>
#include <vector>

class OptimoStateProvider {
public:
  static OptimoStateProvider *GetStateProvider();
  ~OptimoStateProvider() = default;

  // servo dt should be set outside of controller
  double servo_dt_;
  int data_save_freq_;

  int count_;
  double current_time_;

  Eigen::VectorXd nominal_jpos_;

  Eigen::Isometry3d des_ee_iso_;

  Eigen::Matrix3d rot_world_local_;

  bool b_f1_contact_;
  bool b_f2_contact_;
  bool b_f3_contact_;

  int state_;
  int prev_state_;

  int planning_id_;

private:
  OptimoStateProvider();
};

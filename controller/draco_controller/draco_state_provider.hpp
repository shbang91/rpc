#pragma once
#include <Eigen/Dense>
#include <vector>

class DracoStateProvider {
public:
  static DracoStateProvider *GetStateProvider();
  ~DracoStateProvider() = default;

  // servo dt should be set outside of controller
  double servo_dt_;
  int data_save_freq_;

  int count_;
  double current_time_;

  // should be set outside of controller
  Eigen::VectorXd nominal_jpos_;

  // used in pos estimate in estimator module
  int stance_foot_;
  int prev_stance_foot_;

  Eigen::Matrix3d rot_world_local_;

  Eigen::Vector3d dcm_;
  Eigen::Vector3d prev_dcm_;
  Eigen::Vector3d dcm_vel_;

  bool b_lf_contact_;
  bool b_rf_contact_;

  Eigen::Vector3d com_vel_est_;

  int state_;
  int prev_state_;

  bool b_use_base_height_;

  double des_com_height_;
  Eigen::Quaterniond des_torso_quat_;

  int planning_id_;

  std::vector<int> floating_base_jidx_;

  Eigen::Vector3d cam_est_;

private:
  DracoStateProvider();
};

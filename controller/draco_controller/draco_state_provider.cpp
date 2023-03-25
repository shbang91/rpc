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

  servo_dt_ = 0.00125;
  data_save_freq_ = 1;

  count_ = 0;
  current_time_ = 0.;

  stance_foot_ = draco_link::l_foot_contact;
  prev_stance_foot_ = draco_link::l_foot_contact;

  dcm_.setZero();
  prev_dcm_.setZero();
  dcm_vel_.setZero();

  b_lf_contact_ = true;
  b_rf_contact_ = true;

  com_vel_est_.setZero();

  state_ = draco_states::kInitialize;
  prev_state_ = draco_states::kInitialize;

  b_use_base_height_ = false;
  b_use_kf_state_estimator_ = false;

  des_com_height_ = 0.;
  des_torso_quat_ = Eigen::Quaterniond::Identity();

  planning_id_ = 0;

  floating_base_jidx_ = {0, 1, 2, 3, 4, 5};

  cam_est_ = Eigen::Vector3d::Zero();
}

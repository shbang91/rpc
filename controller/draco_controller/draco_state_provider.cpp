#include "controller/draco_controller/draco_state_provider.hpp"
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

  rot_world_local_ = Eigen::Matrix3d::Identity();

  dcm_.setZero();
  prev_dcm_.setZero();
  dcm_vel_.setZero();

  b_lf_contact_ = true;
  b_rf_contact_ = true;
  b_request_change_swing_leg_ = false;
  b_swing_leg_ = end_effector::LFoot;

  com_vel_est_.setZero();

  state_ = 1; // draco_states::kInitialize or draco_states::wbic::kInitialize
  prev_state_ =
      1; // draco_states::kInitialize or draco_states::wbic::kInitialize

  b_use_base_height_ = false;

  des_com_height_ = 0.;
  // des_body_height_ = 0.;
  des_torso_quat_ = Eigen::Quaterniond::Identity();

  planning_id_ = 0;

  floating_base_jidx_ = {0, 1, 2, 3, 4, 5};

  cam_est_ = Eigen::Vector3d::Zero();

  rot_world_local_ = Eigen::Matrix3d::Identity();
  wbo_ypr_ = Eigen::Vector3d::Zero();
  wbo_ang_vel_ = Eigen::Vector3d::Zero();
  wbo_des_ = Eigen::VectorXd::Zero(4);

  // gripper
  b_recv_gripper_cmd_ = false;
  gripper_pos_cmd_["left"] = 0.0;
  gripper_pos_cmd_["right"] = 0.0;
}

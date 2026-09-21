#include "controller/optimo_controller/optimo_state_provider.hpp"
#include "controller/optimo_controller/optimo_control_architecture.hpp"
#include "controller/optimo_controller/optimo_definition.hpp"
#include "util/util.hpp"

OptimoStateProvider *OptimoStateProvider::GetStateProvider() {
  static OptimoStateProvider state_provider;
  return &state_provider;
}

OptimoStateProvider::OptimoStateProvider() {
  util::PrettyConstructor(1, "OptimoStateProvider");

  servo_dt_ = 0.001;
  data_save_freq_ = 1;

  count_ = 0;
  current_time_ = 0.;

  b_f1_contact_ = false;
  b_f2_contact_ = false;
  b_f3_contact_ = false;

  state_ = optimo_states::kInitialize;
  prev_state_ = optimo_states::kInitialize;

  des_ee_iso_ = Eigen::Isometry3d::Identity();

  planning_id_ = 0;

  rot_world_local_ = Eigen::Matrix3d::Identity();
}

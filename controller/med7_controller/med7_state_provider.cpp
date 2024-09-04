#include "controller/med7_controller/med7_state_provider.hpp"
#include "controller/med7_controller/med7_control_architecture.hpp"
#include "controller/med7_controller/med7_definition.hpp"
#include "util/util.hpp"

Med7StateProvider *Med7StateProvider::GetStateProvider() {
  static Med7StateProvider state_provider;
  return &state_provider;
}

Med7StateProvider::Med7StateProvider() {
    util::PrettyConstructor(1, "Med7StateProvider");

    servo_dt_ = 0.001;
    data_save_freq_ = 1;

    count_ = 0;
    current_time_ = 0.;

    state_ = med7_states::kInitialize;
    prev_state_ = med7_states::kInitialize;

    des_ee_iso_ = Eigen::Isometry3d::Identity();

    planning_id_ = 0;

    rot_world_local_ = Eigen::Matrix3d::Identity();
}

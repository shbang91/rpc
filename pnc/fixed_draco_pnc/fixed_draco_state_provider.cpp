#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"
#include "util/util.hpp"

FixedDracoStateProvider *FixedDracoStateProvider::GetStateProvider() {
  static FixedDracoStateProvider state_provider;
  return &state_provider;
}

FixedDracoStateProvider::FixedDracoStateProvider() {
  util::PrettyConstructor(1, "FixedDracoStateProvider");
  servo_dt = 0.001;
  current_time = 0.;

  state = 0;
  prev_state = 0;
}

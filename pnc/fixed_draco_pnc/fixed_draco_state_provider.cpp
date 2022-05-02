#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"

FixedDracoStateProvider *FixedDracoStateProvider::GetStateProvider(){
    static FixedDracoStateProvider state_provider;
    return &state_provider; 
}

FixedDracoStateProvider::FixedDracoStateProvider(){
    servo_dt_ = 0.001;
    current_time_ = 0.;
}

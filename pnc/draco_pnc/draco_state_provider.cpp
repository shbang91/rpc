#include "pnc/draco_pnc/draco_state_provider.hpp"

DracoStateProvider *DracoStateProvider::GetStateProvider(){
    static DracoStateProvider state_provider;
    return &state_provider; 
}

DracoStateProvider::DracoStateProvider(){
    servo_dt_ = 0.001;
    current_time_ = 0.;

    stance_foot_ = "l_foot_contact";
    prev_stance_foot_ = "l_foot_contact";

    dcm_.setZero();
    prev_dcm_.setZero();
    dcm_vel_.setZero();

    b_lf_contact_ = true;
    b_rf_contact_ = true;

}

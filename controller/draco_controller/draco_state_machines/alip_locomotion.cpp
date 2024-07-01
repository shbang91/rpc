#include "controller/draco_controller/draco_state_machines/alip_locomotion.hpp"
#include "util/util.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/whole_body_controller/managers/alipmpc_trajectory_manager.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include <fstream>

//the state will be the following; 
//First visit: the thing would be do a single step since so i start moving
//One step(): will be the main loop. While not interrupt do (at the moment no interrupt,will walk indifinitely)
//Get command will call one step one time but will perform multiple steps. controller_->GetCommand will be performed inside state_machine
AlipLocomotion::AlipLocomotion(StateId state_id, PinocchioRobotSystem *robot,
                            DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch){

    //util::PrettyConstructor(2, "AlipLocomotion");

    sp_ = DracoStateProvider::GetStateProvider();


    first_ever = true;
    new_leg = false;

    file1.open(THIS_COM "/test/alip/LandTime.txt", std::fstream::out);
}


void AlipLocomotion::FirstVisit(){  //represents when MPC computation
  if(first_ever) {
    stance_leg = sp_->initial_stance_leg_;
    state_machine_start_time_ = sp_->current_time_;
    first_ever = false;
    if (stance_leg == 1) {
      ctrl_arch_->alip_tm_->SetSwingFootStart(robot_->GetLinkIsometry(draco_link::l_foot_contact).translation());
    } else {
      ctrl_arch_->alip_tm_->SetSwingFootStart(robot_->GetLinkIsometry(draco_link::r_foot_contact).translation());
    }
    ctrl_arch_->alip_tm_->initializeOri();
    ctrl_arch_->alip_tm_->setNewOri(sp_->des_com_yaw_);
    sp_->des_end_torso_iso_ = ctrl_arch_->alip_tm_->Get_des_end_torso_iso();
    new_leg = true;
  }
  else if (new_leg) {
    state_machine_start_time_ = sp_->current_time_;
    ctrl_arch_->alip_tm_->setNewOri(sp_->des_com_yaw_);
    sp_->des_end_torso_iso_ = ctrl_arch_->alip_tm_->Get_des_end_torso_iso();
  }

  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  Tr = Ts - state_machine_time_;
  sp_->Tr_ = Tr;




  ctrl_arch_->alip_tm_->MpcSolutions(Tr, stance_leg, sp_->Lx_offset_des_, sp_->Ly_des_, sp_->des_com_yaw_,
                                        sp_->kx_, sp_->ky_, sp_->mu_, new_leg);

  if (sp_->des_com_yaw_ != 0) ctrl_arch_->alip_tm_->turning_self_collision();

  sp_->full_policy_ = ctrl_arch_->alip_tm_->add_residual_rl_action(sp_->res_rl_action_);

  
  ctrl_arch_->alip_tm_->GenerateTrajs(Tr, new_leg);
  new_leg = false;

  if (verbose){
    //ctrl_arch_->alip_tm_->saveTrajectories(0, Ts/20, Ts);
    //util::PrettyConstructor(3, "Trajectories saved");
  }
  if (stance_leg == 1) {
    sp_->b_lf_contact_ = false;
    sp_->b_rf_contact_ = true;

    ctrl_arch_->tci_container_->contact_map_["rf_contact"]->SetMaxFz(rf_z_MAX_); //rf_z_MAX_);
    ctrl_arch_->tci_container_->contact_map_["lf_contact"]->SetMaxFz(rf_z_max_); //rf_z_max_);
  }
  else {
    ctrl_arch_->tci_container_->contact_map_["lf_contact"]->SetMaxFz(rf_z_MAX_); //rf_z_MAX_);
    ctrl_arch_->tci_container_->contact_map_["rf_contact"]->SetMaxFz(rf_z_max_); //rf_z_max_);
    sp_->b_rf_contact_ = false;
    sp_->b_lf_contact_ = true;

  }

  /*
  if (verbose){
    ctrl_arch_->alip_tm_->saveTrajectories(0, Ts/20, Ts);
    util::PrettyConstructor(3, "Trajectories saved");
  }
  */


}
void AlipLocomotion::OneStep(){
    state_machine_time_ = sp_->current_time_ - state_machine_start_time_;
    double t = state_machine_time_; //+ Tr - Ts; //time since Tr was computed (mpc solution and trajectory generated)
    ctrl_arch_->alip_tm_->UpdateDesired(t);
}
void AlipLocomotion::LastVisit(){

}
bool AlipLocomotion::EndOfState(){

}


StateId AlipLocomotion::GetNextState() {
  return draco_states::kDoubleSupportBalance;
}





bool AlipLocomotion::SwitchLeg(){  //ahora asume que tocamos en Tr o antes. Que pasa si se atrasa?
  bool switch_leg = false;

  //if (Tr < 0.5*Ts) {   //have to add a restriction to not have consecutives
  if (sp_->current_time_ - state_machine_start_time_ > 0.5*Ts){
    //if ((stance_leg == 1) && (sp_->b_lf_contact_)){  //right stance, left swing
    if((stance_leg == 1) && (robot_->GetLinkIsometry(draco_link::l_foot_contact).translation()(2) < 0.00005)){
      //if (verbose){
        //util::PrettyConstructor(2, "Switch Leg AlipLocomotion true ");
        //std::cout << "Right stance to left" << " | Tr:" << Tr << "  | state machine time:" << state_machine_time_  <<std::endl;
      //}
      stance_leg *= -1;
      //ctrl_arch_->alip_tm_->RToLstance();
      //update the force managers
      switch_leg = true;

      state_machine_start_time_ = sp_->current_time_; //this should go to new leg

      ctrl_arch_->tci_container_->contact_map_["lf_contact"]->SetMaxFz(rf_z_MAX_);
      new_leg = true;


      //ctrl_arch_->tci_container_->task_map_["rf_pos"]->SetMaxFz(0.01);
      ctrl_arch_->alip_tm_->SetSwingFootStart(robot_->GetLinkIsometry(draco_link::r_foot_contact).translation());
      ctrl_arch_->alip_tm_->saveDoubleStanceFoot();


    }  //else if((stance_leg == -1) && (sp_->b_rf_contact_)){
    else if((stance_leg == -1) && (robot_->GetLinkIsometry(draco_link::r_foot_contact).translation()(2) < 0.00005)){
      //if (verbose){
        //util::PrettyConstructor(2, "Switch Leg AlipLocomotion true ");
        //std::cout << "Left stance to right" << " | Tr:" << Tr << "  | state machine time:" << state_machine_time_ <<std::endl;
      //}
      stance_leg *=-1;

      switch_leg = true;

      state_machine_start_time_ = sp_->current_time_; //same thing than before

      ctrl_arch_->tci_container_->contact_map_["rf_contact"]->SetMaxFz(rf_z_MAX_);
      new_leg = true;

      ctrl_arch_->alip_tm_->SetSwingFootStart(robot_->GetLinkIsometry(draco_link::l_foot_contact).translation());
      ctrl_arch_->alip_tm_->saveDoubleStanceFoot();
    }
  }
  sp_->stance_leg_ = stance_leg;
  sp_->Ts_ = Ts;
  sp_->Tr_ = Tr;
  
  if ((verbose) && (switch_leg)){ 
    file1 << sp_->current_time_ << endl; 
  }
  sp_->stance_leg_ = stance_leg;
  sp_->Ts_ = Ts;
  sp_->Tr_ = Tr;
  
  return switch_leg;
}

void AlipLocomotion::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "swing_height", swing_height_);
    util::ReadParameter(node, "Ts", Ts);
    util::ReadParameter(node, "stance_leg", stance_leg);
    util::ReadParameter(node, "rf_z_MAX", rf_z_MAX_);
    util::ReadParameter(node, "rf_z_max", rf_z_max_);
    util::ReadParameter(node, "stance_leg", sp_->initial_stance_leg_);
    util::ReadParameter(node, "total_mass", sp_->mass_);
    util::ReadParameter(node, "verbose", verbose);

  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}


int AlipLocomotion::GetStance_leg(){return stance_leg;}

void AlipLocomotion::Reset(){
    first_ever = true;
    new_leg = false;
}


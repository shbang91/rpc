#include "controller/draco_controller/draco_state_machines/double_support_balance.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"

DoubleSupportBalance::DoubleSupportBalance(const StateId state_id,
                                           PinocchioRobotSystem *robot,
                                           DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch) {
  //util::PrettyConstructor(2, "DoubleSupportBalance");

  try {
    YAML::Node cfg =
        YAML::LoadFile(THIS_COM "config/draco/pnc.yaml"); // get yaml node
    b_use_fixed_foot_pos_ = util::ReadParameter<bool>(
        cfg["state_machine"], "b_use_const_desired_foot_pos");
  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }

  sp_ = DracoStateProvider::GetStateProvider();
  nominal_lfoot_iso_.setIdentity();
  nominal_rfoot_iso_.setIdentity();
}

void DoubleSupportBalance::FirstVisit() {
  //std::cout << "draco_states: kDoubleSupportBalance" << std::endl;
  state_machine_start_time_ = sp_->current_time_;

  // reset flags

  // set current foot position as nominal (desired) for rest of this state
  nominal_lfoot_iso_ = robot_->GetLinkIsometry(draco_link::l_foot_contact);
  nominal_rfoot_iso_ = robot_->GetLinkIsometry(draco_link::r_foot_contact);
  FootStep::MakeHorizontal(nominal_lfoot_iso_);
  FootStep::MakeHorizontal(nominal_rfoot_iso_);
}

void DoubleSupportBalance::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // update foot pose task update
  if (b_use_fixed_foot_pos_) {
    ctrl_arch_->lf_SE3_tm_->UseNominal(nominal_lfoot_iso_);
    ctrl_arch_->rf_SE3_tm_->UseNominal(nominal_rfoot_iso_);
  } else {
    ctrl_arch_->lf_SE3_tm_->UseCurrent();
    ctrl_arch_->rf_SE3_tm_->UseCurrent();
  }
}

bool DoubleSupportBalance::EndOfState() {
  if (state_machine_time_ > 0.1) return true;
  return false;
}

void DoubleSupportBalance::LastVisit() {
  state_machine_time_ = 0.;

  if (sp_->b_use_base_height_)
    sp_->des_com_height_ = robot_->GetRobotComPos()[2];
  /*
  std::cout << "-----------------------------------------" << std::endl;
  std::cout << "des com height: " << sp_->des_com_height_ << std::endl;
  std::cout << "-----------------------------------------" << std::endl;
  */
  Eigen::Isometry3d torso_iso =
      robot_->GetLinkIsometry(draco_link::torso_com_link);
  FootStep::MakeHorizontal(torso_iso);
  sp_->rot_world_local_ = torso_iso.linear();
}

StateId DoubleSupportBalance::GetNextState() {
    return draco_states::AlipLocomotion;
  }


void DoubleSupportBalance::SetParameters(const YAML::Node &node) {}
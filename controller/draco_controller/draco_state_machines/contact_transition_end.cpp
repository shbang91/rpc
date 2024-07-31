#include "controller/draco_controller/draco_state_machines/contact_transition_end.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/reaction_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "double_support_stand_up.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"

ContactTransitionEnd::ContactTransitionEnd(StateId state_id,
                                           PinocchioRobotSystem *robot,
                                           DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch) {

  if (state_id_ == draco_states::kLFContactTransitionEnd)
    util::PrettyConstructor(2, "LFContactTransitionEnd");
  else if (state_id_ == draco_states::kRFContactTransitionEnd)
    util::PrettyConstructor(2, "RFContactTransitionEnd");

  try {
    YAML::Node cfg =
        YAML::LoadFile(THIS_COM "config/draco/pnc.yaml"); // get yaml node
    b_use_fixed_foot_pos_ = util::ReadParameter<bool>(
        cfg["state_machine"], "b_use_const_desired_foot_pos");
  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }

  nominal_lfoot_iso_.setIdentity();
  nominal_rfoot_iso_.setIdentity();
  sp_ = DracoStateProvider::GetStateProvider();
}

void ContactTransitionEnd::FirstVisit() {
  state_machine_start_time_ = sp_->current_time_;

  end_time_ =
      ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampDownTime();

  if (state_id_ == draco_states::kLFContactTransitionEnd) {
    std::cout << "draco_States::kLFContactTransitionEnd" << std::endl;
    // =====================================================================
    // task hierarchy manager initialize
    // =====================================================================
    ctrl_arch_->lf_pos_hm_->InitializeRampToMin(end_time_);
    ctrl_arch_->lf_ori_hm_->InitializeRampToMin(end_time_);

    // =====================================================================
    // contact max normal force manager initialize
    // =====================================================================
    ctrl_arch_->lf_max_normal_froce_tm_->InitializeRampToMin(end_time_);

    // =====================================================================
    // set reference desired reaction force initialize
    // =====================================================================
    Eigen::VectorXd zero_force = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd max_force_z = Eigen::VectorXd::Zero(6);
    ctrl_arch_->lf_force_tm_->InitializeInterpolation(zero_force, end_time_);
    max_force_z[5] = kGravity * robot_->GetTotalMass();
    ctrl_arch_->rf_force_tm_->InitializeInterpolation(max_force_z, end_time_);
  } else if (state_id_ == draco_states::kRFContactTransitionEnd) {
    std::cout << "draco_states::kRFContactTransitionEnd" << std::endl;
    // =====================================================================
    // task hierarchy manager initialize
    // =====================================================================
    ctrl_arch_->rf_pos_hm_->InitializeRampToMin(end_time_);
    ctrl_arch_->rf_ori_hm_->InitializeRampToMin(end_time_);

    // =====================================================================
    // contact max normal force manager initialize
    // =====================================================================
    ctrl_arch_->rf_max_normal_froce_tm_->InitializeRampToMin(end_time_);

    // =====================================================================
    // set reference desired reaction force initialize
    // =====================================================================
    Eigen::VectorXd zero_force = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd max_force_z = Eigen::VectorXd::Zero(6);
    ctrl_arch_->rf_force_tm_->InitializeInterpolation(zero_force, end_time_);
    max_force_z[5] = kGravity * robot_->GetTotalMass();
    ctrl_arch_->lf_force_tm_->InitializeInterpolation(max_force_z, end_time_);
  }

  // set current foot position as nominal (desired) for rest of this state
  nominal_lfoot_iso_ = robot_->GetLinkIsometry(draco_link::l_foot_contact);
  nominal_rfoot_iso_ = robot_->GetLinkIsometry(draco_link::r_foot_contact);
}

void ContactTransitionEnd::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // com pos & torso_ori task update
  ctrl_arch_->dcm_tm_->UpdateDesired(sp_->current_time_);

  // foot pose task update
  if (b_use_fixed_foot_pos_) {
    ctrl_arch_->lf_SE3_tm_->UseNominal(nominal_lfoot_iso_);
    ctrl_arch_->rf_SE3_tm_->UseNominal(nominal_rfoot_iso_);
  } else {
    ctrl_arch_->lf_SE3_tm_->UseCurrent();
    ctrl_arch_->rf_SE3_tm_->UseCurrent();
  }

  // contact task & force update
  if (state_id_ == draco_states::kLFContactTransitionEnd) {
    ctrl_arch_->lf_max_normal_froce_tm_->UpdateRampToMin(state_machine_time_);
    ctrl_arch_->lf_pos_hm_->UpdateRampToMin(state_machine_time_);
    ctrl_arch_->lf_ori_hm_->UpdateRampToMin(state_machine_time_);
  } else if (state_id_ == draco_states::kRFContactTransitionEnd) {
    ctrl_arch_->rf_max_normal_froce_tm_->UpdateRampToMin(state_machine_time_);
    ctrl_arch_->rf_pos_hm_->UpdateRampToMin(state_machine_time_);
    ctrl_arch_->rf_ori_hm_->UpdateRampToMin(state_machine_time_);
  }

  // update force traj manager
  ctrl_arch_->lf_force_tm_->UpdateDesired(state_machine_time_);
  ctrl_arch_->rf_force_tm_->UpdateDesired(state_machine_time_);
}

bool ContactTransitionEnd::EndOfState() {
  return state_machine_time_ > end_time_ ? true : false;
}

void ContactTransitionEnd::LastVisit() { state_machine_time_ = 0.; }

StateId ContactTransitionEnd::GetNextState() {
  if (state_id_ == draco_states::kLFContactTransitionEnd)
    return draco_states::kLFSingleSupportSwing;
  else if (state_id_ == draco_states::kRFContactTransitionEnd)
    return draco_states::kRFSingleSupportSwing;
}

void ContactTransitionEnd::SetParameters(const YAML::Node &node) {}

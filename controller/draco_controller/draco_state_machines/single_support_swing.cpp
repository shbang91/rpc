#include "controller/draco_controller/draco_state_machines/single_support_swing.hpp"
#include "configuration.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"

SingleSupportSwing::SingleSupportSwing(StateId state_id,
                                       PinocchioRobotSystem *robot,
                                       DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch), swing_height_(0.) {

  if (state_id_ == draco_states::kLFSingleSupportSwing)
    util::PrettyConstructor(2, "kLFSingleSupportSwing");
  else if (state_id_ == draco_states::kRFSingleSupportSwing)
    util::PrettyConstructor(2, "kRFSingleSupportSwing");

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

void SingleSupportSwing::FirstVisit() {
  state_machine_start_time_ = sp_->current_time_;
  end_time_ = ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetSwingTime();

  int curr_foot_step_idx = ctrl_arch_->dcm_tm_->GetCurrentFootStepIdx();

  if (state_id_ == draco_states::kLFSingleSupportSwing) {
    std::cout << "draco_states::kLFSingleSupportSwing" << std::endl;

    Eigen::Isometry3d curr_lfoot_iso =
        robot_->GetLinkIsometry(draco_link::l_foot_contact);
    FootStep::MakeHorizontal(curr_lfoot_iso);

    FootStep target_lfoot_step =
        ctrl_arch_->dcm_tm_->GetFootStepList()[curr_foot_step_idx];
    Eigen::Isometry3d target_lfoot_iso =
        FootStep::MakeIsometry(target_lfoot_step);

    // foot traj initialize
    ctrl_arch_->lf_SE3_tm_->InitializeSwingTrajectory(
        curr_lfoot_iso, target_lfoot_iso, swing_height_, end_time_);
    nominal_rfoot_iso_ = robot_->GetLinkIsometry(draco_link::r_foot_contact);

    sp_->b_lf_contact_ = false;
    sp_->b_request_change_swing_leg_ = true;
    sp_->b_swing_leg_ = end_effector::LFoot;

  } else if (state_id_ == draco_states::kRFSingleSupportSwing) {
    std::cout << "draco_states::kRFSingleSupportSwing" << std::endl;

    Eigen::Isometry3d curr_rfoot_iso =
        robot_->GetLinkIsometry(draco_link::r_foot_contact);
    FootStep::MakeHorizontal(curr_rfoot_iso);

    FootStep target_rfoot_step =
        ctrl_arch_->dcm_tm_->GetFootStepList()[curr_foot_step_idx];
    Eigen::Isometry3d target_rfoot_iso =
        FootStep::MakeIsometry(target_rfoot_step);

    // foot traj initialize
    ctrl_arch_->rf_SE3_tm_->InitializeSwingTrajectory(
        curr_rfoot_iso, target_rfoot_iso, swing_height_, end_time_);
    nominal_lfoot_iso_ = robot_->GetLinkIsometry(draco_link::l_foot_contact);

    sp_->b_rf_contact_ = false;
    sp_->b_request_change_swing_leg_ = true;
    sp_->b_swing_leg_ = end_effector::RFoot;
  }
}

void SingleSupportSwing::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // com pos & torso ori task update
  ctrl_arch_->dcm_tm_->UpdateDesired(sp_->current_time_);

  // foot task update
  if (state_id_ == draco_states::kLFSingleSupportSwing) {
    ctrl_arch_->lf_SE3_tm_->UpdateDesired(state_machine_time_);
    if (b_use_fixed_foot_pos_) {
      ctrl_arch_->rf_SE3_tm_->UseNominal(nominal_rfoot_iso_);
    } else {
      ctrl_arch_->rf_SE3_tm_->UseCurrent();
    }
  } else if (state_id_ == draco_states::kRFSingleSupportSwing) {
    ctrl_arch_->rf_SE3_tm_->UpdateDesired(state_machine_time_);
    if (b_use_fixed_foot_pos_) {
      ctrl_arch_->lf_SE3_tm_->UseNominal(nominal_lfoot_iso_);
    } else {
      ctrl_arch_->lf_SE3_tm_->UseCurrent();
    }
  }
}

void SingleSupportSwing::LastVisit() {
  ctrl_arch_->dcm_tm_->IncrementStepIndex();
  state_machine_time_ = 0.;
}

bool SingleSupportSwing::EndOfState() {
  bool b_early_contact = false;
  if ((state_id_ == draco_states::kLFSingleSupportSwing) &&
      (sp_->b_lf_contact_)) {
    b_early_contact = true;
    std::cout << "Early Touchdown [LFoot]" << std::endl;
  } else if ((state_id_ == draco_states::kRFSingleSupportSwing) &&
             (sp_->b_rf_contact_)) {
    b_early_contact = true;
    std::cout << "Early Touchdown [RFoot]" << std::endl;
  }

  return (state_machine_time_ > end_time_) || b_early_contact ? true : false;
}

StateId SingleSupportSwing::GetNextState() {
  if (state_id_ == draco_states::kLFSingleSupportSwing)
    return draco_states::kRFContactTransitionStart;
  else if (state_id_ == draco_states::kRFSingleSupportSwing)
    return draco_states::kLFContactTransitionStart;
}

void SingleSupportSwing::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "swing_height", swing_height_);

  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

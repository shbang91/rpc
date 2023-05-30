#include "controller/draco_controller/draco_state_machines/contact_transition_start.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "controller/whole_body_controller/task.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"
#include "controller/whole_body_controller/managers/reaction_force_trajectory_manager.hpp"
#include "double_support_stand_up.hpp"

ContactTransitionStart::ContactTransitionStart(
    StateId state_id, PinocchioRobotSystem *robot,
    DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch) {

  if (state_id_ == draco_states::kLFContactTransitionStart)
    util::PrettyConstructor(2, "LFContactTransitionStart");
  else if (state_id_ == draco_states::kRFContactTransitionStart)
    util::PrettyConstructor(2, "RFContactTransitionStart");

  try {
    YAML::Node cfg =
            YAML::LoadFile(THIS_COM "config/draco/pnc.yaml"); // get yaml node
    b_use_fixed_foot_pos_ = util::ReadParameter<bool>(cfg["state_machine"],
                                                "b_use_const_desired_foot_pos");
  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }

  nominal_lfoot_iso_.setIdentity();
  nominal_rfoot_iso_.setIdentity();
  sp_ = DracoStateProvider::GetStateProvider();
}

void ContactTransitionStart::FirstVisit() {
  state_machine_start_time_ = sp_->current_time_;

  if (state_id_ == draco_states::kLFContactTransitionStart) {
    // stance foot lfoot
    std::cout << "draco_states::kLFContactTransitionStart" << std::endl;
    // =====================================================================
    // task hierarchy manager initialize
    // =====================================================================
    ctrl_arch_->rf_pos_hm_->InitializeRampToMax(
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());
    ctrl_arch_->rf_ori_hm_->InitializeRampToMax(
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());

    // =====================================================================
    // contact max normal force manager initialize
    // =====================================================================
    ctrl_arch_->rf_max_normal_froce_tm_->InitializeRampToMax(
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());

    sp_->b_rf_contact_ = true;

  } else {
    // stance foot rfoot
    std::cout << "draco_states::kRFContactTransitionStart" << std::endl;
    // =====================================================================
    // task hierarchy manager initialize
    // =====================================================================
    ctrl_arch_->lf_pos_hm_->InitializeRampToMax(
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());
    ctrl_arch_->lf_ori_hm_->InitializeRampToMax(
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());
    // =====================================================================
    // contact max normal force manager initialize
    // =====================================================================
    ctrl_arch_->lf_max_normal_froce_tm_->InitializeRampToMax(
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());

    sp_->b_lf_contact_ = true;
  }

  // =====================================================================
  // dcm planner initialize
  // =====================================================================
  if (ctrl_arch_->dcm_tm_->NoRemainingSteps()){
    end_time_ =
            ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetFinalContactTransferTime();
    rf_end_time_ = 0.5 * end_time_;
  }
  else {
    // Eigen::Quaterniond init_torso_quat(
    // robot_->GetLinkIsometry(draco_link::torso_com_link).linear());
    // Eigen::Vector3d init_dcm_pos = sp_->dcm_;
    // Eigen::Vector3d init_dcm_vel = sp_->dcm_vel_;
    // if (sp_->b_use_base_height_) {
    // init_dcm_pos[2] =
    // robot_->GetLinkIsometry(draco_link::torso_com_link).translation()[2];
    // init_dcm_vel[2] =
    // robot_->GetLinkSpatialVel(draco_link::torso_com_link)[5];
    //}

    if (sp_->prev_state_ == draco_states::kDoubleSupportBalance) {
      Eigen::Vector3d ini_des_dcm_pos = Eigen::Vector3d::Zero();
      ini_des_dcm_pos
          << ctrl_arch_->tci_container_->task_map_["com_xy_task"]->DesiredPos(),
          ctrl_arch_->tci_container_->task_map_["com_z_task"]->DesiredPos();

      Eigen::Vector3d ini_des_dcm_vel = Eigen::Vector3d::Zero();
      ini_des_dcm_vel
          << ctrl_arch_->tci_container_->task_map_["com_xy_task"]->DesiredVel(),
          ctrl_arch_->tci_container_->task_map_["com_z_task"]->DesiredVel();

      std::cout << "ini des dcm pos: " << ini_des_dcm_pos.transpose()
                << std::endl;
      std::cout << "ini des dcm vel: " << ini_des_dcm_vel.transpose()
                << std::endl;

      if (sp_->b_use_base_height_)
        ini_des_dcm_pos[2] = sp_->des_com_height_;

      std::cout << "ini des dcm pos after changes: "
                << ini_des_dcm_pos.transpose() << std::endl;

      Eigen::Quaterniond ini_torso_quat = sp_->des_torso_quat_;

      ctrl_arch_->dcm_tm_->Initialize(
          sp_->current_time_, dcm_transfer_type::kInitial, ini_torso_quat,
          ini_des_dcm_pos, ini_des_dcm_vel);
      end_time_ =
          ctrl_arch_->dcm_tm_->GetDCMPlanner()
              ->GetInitialContactTransferTime() -
          ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampDownTime();

      ctrl_arch_->dcm_tm_->GetDCMPlanner()->SaveSolution(
          std::to_string(sp_->planning_id_));
      sp_->planning_id_ += 1;
    } else {
      // TODO: replanning
      // TODO: consider torso ang vel
      int transfer_type = dcm_transfer_type::kMidStep;
      // ctrl_arch_->dcm_tm_->Initialize(
      // sp_->current_time_, dcm_transfer_type::kMidStep, init_torso_quat,
      // init_dcm_pos, init_dcm_vel);
      end_time_ =
          ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime();
    }
    rf_end_time_ = end_time_;
  }

  // set reference desired reaction force
  Eigen::VectorXd mid_max_force_z = Eigen::VectorXd::Zero(6);
  mid_max_force_z[5] = kGravity * robot_->GetTotalMass() / 2.;
  ctrl_arch_->lf_force_tm_->InitializeInterpolation(mid_max_force_z, rf_end_time_);
  ctrl_arch_->rf_force_tm_->InitializeInterpolation(mid_max_force_z, rf_end_time_);

  // set current foot position as nominal (desired) for rest of this state
  nominal_lfoot_iso_ = robot_->GetLinkIsometry(draco_link::l_foot_contact);
  nominal_rfoot_iso_ = robot_->GetLinkIsometry(draco_link::r_foot_contact);
}

void ContactTransitionStart::OneStep() {
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

  // contact max normal force & foot task hierarchy update
  if (state_id_ == draco_states::kLFContactTransitionStart) {
    ctrl_arch_->rf_max_normal_froce_tm_->UpdateRampToMax(state_machine_time_);
    ctrl_arch_->rf_pos_hm_->UpdateRampToMax(state_machine_time_);
    ctrl_arch_->rf_ori_hm_->UpdateRampToMax(state_machine_time_);
  } else {
    ctrl_arch_->lf_max_normal_froce_tm_->UpdateRampToMax(state_machine_time_);
    ctrl_arch_->lf_pos_hm_->UpdateRampToMax(state_machine_time_);
    ctrl_arch_->lf_ori_hm_->UpdateRampToMax(state_machine_time_);
  }

  // update force traj manager
  ctrl_arch_->lf_force_tm_->UpdateDesired(state_machine_time_);
  ctrl_arch_->rf_force_tm_->UpdateDesired(state_machine_time_);
}

bool ContactTransitionStart::EndOfState() {
  return state_machine_time_ > end_time_ ? true : false;
}

void ContactTransitionStart::LastVisit() { state_machine_time_ = 0.; }

StateId ContactTransitionStart::GetNextState() {
  if (ctrl_arch_->dcm_tm_->NoRemainingSteps())
    return draco_states::kDoubleSupportBalance;
  else {
    if (state_id_ == draco_states::kLFContactTransitionStart) {
      sp_->stance_foot_ = draco_link::r_foot_contact;
      return draco_states::kLFContactTransitionEnd;

    } else {
      sp_->stance_foot_ = draco_link::l_foot_contact;
      return draco_states::kRFContactTransitionEnd;
    }
  }
}

void ContactTransitionStart::SetParameters(const YAML::Node &node) {}

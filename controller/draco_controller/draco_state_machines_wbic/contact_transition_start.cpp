#include "controller/draco_controller/draco_state_machines_wbic/contact_transition_start.hpp"
#include "controller/draco_controller/draco_control_architecture_wbic.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/contact.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/qp_params_manager.hpp"
#include "controller/whole_body_controller/task.hpp"
#include "controller/whole_body_controller/wbic/wbic.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"

ContactTransitionStart_WBIC::ContactTransitionStart_WBIC(
    StateId state_id, PinocchioRobotSystem *robot,
    DracoControlArchitecture_WBIC *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch) {

  if (state_id_ == draco_states_wbic::kLFContactTransitionStart)
    util::PrettyConstructor(2, "LFContactTransitionStart_WBIC");
  else if (state_id_ == draco_states_wbic::kRFContactTransitionStart)
    util::PrettyConstructor(2, "RFContactTransitionStart_WBIC");

  // contact_vector_.clear();
  // contact_vector_.push_back(
  // ctrl_arch_->tci_container_->contact_map_["lf_contact"]);
  // contact_vector_.push_back(
  // ctrl_arch_->tci_container_->contact_map_["rf_contact"]);

  // task_vector_.clear();
  // task_vector_.push_back(ctrl_arch_->tci_container_->task_map_["com_xy_task"]);
  // task_vector_.push_back(ctrl_arch_->tci_container_->task_map_["com_z_task"]);
  // task_vector_.push_back(
  // ctrl_arch_->tci_container_->task_map_["torso_ori_task"]);
  // task_vector_.push_back(
  // ctrl_arch_->tci_container_->task_map_["upper_body_task"]);

  sp_ = DracoStateProvider::GetStateProvider();
}

void ContactTransitionStart_WBIC::FirstVisit() {
  state_machine_start_time_ = sp_->current_time_;

  if (state_id_ == draco_states_wbic::kLFContactTransitionStart) {
    // stance foot lfoot & right foot touches ground
    std::cout << "draco_states_wbic::kLFContactTransitionStart" << std::endl;
    // =====================================================================
    // contact max normal force manager initialize
    // =====================================================================
    ctrl_arch_->rf_max_normal_froce_tm_->InitializeRampToMax(
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());

    // change right foot rf QP params
    Eigen::VectorXd target_W_delta_rf =
        ctrl_arch_->tci_container_->qp_params_->W_delta_rf_;
    target_W_delta_rf.tail<6>() = W_delta_rf_right_foot_in_contact_;
    ctrl_arch_->qp_pm_->InitializeWDeltaRfInterpolation(
        target_W_delta_rf,
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());

    // change right foot contact acc QP params
    Eigen::VectorXd target_W_xc_ddot = Eigen::VectorXd::Zero(12);
    target_W_xc_ddot << W_xc_ddot_in_contact_, W_xc_ddot_in_contact_;
    ctrl_arch_->qp_pm_->InitializeWContactInterpolation(
        target_W_xc_ddot,
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());

    sp_->b_rf_contact_ = true;

  } else {
    // stance foot rfoot
    std::cout << "draco_states_wbic::kRFContactTransitionStart" << std::endl;
    // =====================================================================
    // contact max normal force manager initialize
    // =====================================================================
    ctrl_arch_->lf_max_normal_froce_tm_->InitializeRampToMax(
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());

    // change left foot rf QP params
    Eigen::VectorXd target_W_delta_rf =
        ctrl_arch_->tci_container_->qp_params_->W_delta_rf_;
    target_W_delta_rf.head<6>() = W_delta_rf_left_foot_in_contact_;
    ctrl_arch_->qp_pm_->InitializeWDeltaRfInterpolation(
        target_W_delta_rf,
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());

    // change left foot contact acc QP params
    Eigen::VectorXd target_W_xc_ddot = Eigen::VectorXd::Zero(12);
    target_W_xc_ddot << W_xc_ddot_in_contact_, W_xc_ddot_in_contact_;
    ctrl_arch_->qp_pm_->InitializeWContactInterpolation(
        target_W_xc_ddot,
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetNormalForceRampUpTime());

    sp_->b_lf_contact_ = true;
  }

  // build tasks & contacts
  auto &contact_vector = ctrl_arch_->tci_container_->contact_vector_;
  auto &contact_map = ctrl_arch_->tci_container_->contact_map_;
  contact_vector.clear();
  contact_vector.push_back(contact_map["lf_contact"]);
  contact_vector.push_back(contact_map["rf_contact"]);
  Eigen::Isometry3d lfoot_iso =
      robot_->GetLinkIsometry(draco_link::l_foot_contact);
  Eigen::Isometry3d rfoot_iso =
      robot_->GetLinkIsometry(draco_link::r_foot_contact);
  FootStep::MakeHorizontal(lfoot_iso);
  FootStep::MakeHorizontal(rfoot_iso);
  contact_map["lf_contact"]->SetDesiredPos(lfoot_iso.translation());
  contact_map["rf_contact"]->SetDesiredPos(rfoot_iso.translation());
  contact_map["lf_contact"]->SetDesiredOri(
      Eigen::Quaterniond(lfoot_iso.linear()));
  contact_map["rf_contact"]->SetDesiredOri(
      Eigen::Quaterniond(rfoot_iso.linear()));

  auto &task_vector = ctrl_arch_->tci_container_->task_vector_;
  auto &task_map = ctrl_arch_->tci_container_->task_map_;
  task_vector.clear();
  task_vector.push_back(task_map["com_z_task"]);
  task_vector.push_back(task_map["com_xy_task"]);
  task_vector.push_back(task_map["torso_ori_task"]);
  // task_vector.push_back(task_map["wbo_task"]);
  task_vector.push_back(task_map["upper_body_task"]);
  // ctrl_arch_->tci_container_->contact_vector_ = contact_vector_;
  // ctrl_arch_->tci_container_->task_vector_ = task_vector_;

  // =====================================================================
  // dcm planner initialize
  // =====================================================================
  if (ctrl_arch_->dcm_tm_->NoRemainingSteps())
    end_time_ =
        ctrl_arch_->dcm_tm_->GetDCMPlanner()->GetFinalContactTransferTime();
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

    if (sp_->prev_state_ == draco_states_wbic::kDoubleSupportBalance) {
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
  }
}

void ContactTransitionStart_WBIC::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // com pos & torso_ori task update
  ctrl_arch_->dcm_tm_->UpdateDesired(sp_->current_time_);

  // foot pose task update
  // ctrl_arch_->lf_SE3_tm_->UseCurrent();
  // ctrl_arch_->rf_SE3_tm_->UseCurrent();

  // contact max normal force & foot task hierarchy update
  if (state_id_ == draco_states_wbic::kLFContactTransitionStart) {
    ctrl_arch_->rf_max_normal_froce_tm_->UpdateRampToMax(state_machine_time_);
  } else {
    ctrl_arch_->lf_max_normal_froce_tm_->UpdateRampToMax(state_machine_time_);
  }

  // update qp params
  ctrl_arch_->qp_pm_->UpdateWDeltaRfInterpolation(state_machine_time_);
  ctrl_arch_->qp_pm_->UpdateWContactInterpolation(state_machine_time_);
}

bool ContactTransitionStart_WBIC::EndOfState() {
  return state_machine_time_ > end_time_ ? true : false;
}

void ContactTransitionStart_WBIC::LastVisit() { state_machine_time_ = 0.; }

StateId ContactTransitionStart_WBIC::GetNextState() {
  if (ctrl_arch_->dcm_tm_->NoRemainingSteps())
    return draco_states_wbic::kDoubleSupportBalance;
  else {
    if (state_id_ == draco_states_wbic::kLFContactTransitionStart) {
      sp_->stance_foot_ = draco_link::r_foot_contact;
      return draco_states_wbic::kLFContactTransitionEnd;

    } else {
      sp_->stance_foot_ = draco_link::l_foot_contact;
      return draco_states_wbic::kRFContactTransitionEnd;
    }
  }
}

void ContactTransitionStart_WBIC::SetParameters(const YAML::Node &cfg) {
  try {
    // qp params yaml
    util::ReadParameter(cfg["wbc"]["qp"], "W_xc_ddot_in_contact",
                        W_xc_ddot_in_contact_);
    util::ReadParameter(cfg["wbc"]["qp"], "W_delta_rf_left_foot_in_contact",
                        W_delta_rf_left_foot_in_contact_);
    util::ReadParameter(cfg["wbc"]["qp"], "W_delta_rf_right_foot_in_contact",
                        W_delta_rf_right_foot_in_contact_);

  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

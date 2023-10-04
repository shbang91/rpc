#include "controller/draco_controller/draco_state_machines/double_support_stand_up.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_task/draco_com_task.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/reaction_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/wbic/wbic.hpp"
#include "planner/locomotion/dcm_planner/foot_step.hpp"
#include "util/util.hpp"

DoubleSupportStandUp::DoubleSupportStandUp(const StateId state_id,
                                           PinocchioRobotSystem *robot,
                                           DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch), target_height_(0.),
      rf_z_max_interp_duration_(0.), W_delta_qddot_(0.),
      W_xc_ddot_in_contact_(0.),
      W_delta_rf_left_foot_in_contact_(Eigen::VectorXd::Zero(6)),
      W_delta_rf_right_foot_in_contact_(Eigen::VectorXd::Zero(6))
// W_force_rate_of_change_left_foot_(Eigen::VectorXd::Zero(6)),
// W_force_rate_of_change_right_foot_(Eigen::VectorXd::Zero(6)) {
{
  util::PrettyConstructor(2, "DoubleSupportStandUp");

  sp_ = DracoStateProvider::GetStateProvider();
}

void DoubleSupportStandUp::FirstVisit() {
  std::cout << "draco_states::kDoubleSupportStandUp" << std::endl;
  state_machine_start_time_ = sp_->current_time_;

  // set local frame for task gains
  Eigen::Isometry3d stance_foot_iso =
      robot_->GetLinkIsometry(sp_->stance_foot_);
  FootStep::MakeHorizontal(stance_foot_iso);
  sp_->rot_world_local_ = stance_foot_iso.linear();

  // initial com & torso ori setting
  Eigen::Vector3d init_com_pos = robot_->GetRobotComPos();
  if (sp_->b_use_base_height_)
    init_com_pos[2] =
        robot_->GetLinkIsometry(draco_link::torso_com_link).translation()[2];
  Eigen::Matrix3d R_w_torso =
      robot_->GetLinkIsometry(draco_link::torso_com_link).linear();
  Eigen::Quaterniond init_torso_quat(R_w_torso);

  // desired com & torso ori setting
  Eigen::Isometry3d lfoot_iso =
      robot_->GetLinkIsometry(draco_link::l_foot_contact);
  Eigen::Isometry3d rfoot_iso =
      robot_->GetLinkIsometry(draco_link::r_foot_contact);

  FootStep::MakeHorizontal(lfoot_iso);
  FootStep::MakeHorizontal(rfoot_iso);

  Eigen::Vector3d target_com_pos =
      (lfoot_iso.translation() + rfoot_iso.translation()) / 2.;
  target_com_pos[2] = target_height_;

  Eigen::Quaterniond lfoot_quat(lfoot_iso.linear());
  Eigen::Quaterniond rfoot_quat(rfoot_iso.linear());
  Eigen::Quaterniond target_torso_quat = lfoot_quat.slerp(0.5, rfoot_quat);
  sp_->des_torso_quat_ = target_torso_quat;

  // initialize floating trajectory
  ctrl_arch_->floating_base_tm_->InitializeFloatingBaseInterpolation(
      init_com_pos, target_com_pos, init_torso_quat, target_torso_quat,
      end_time_);

  //  increase maximum normal reaction force
  ctrl_arch_->lf_max_normal_froce_tm_->InitializeRampToMax(
      rf_z_max_interp_duration_);
  ctrl_arch_->rf_max_normal_froce_tm_->InitializeRampToMax(
      rf_z_max_interp_duration_);

  // QP params
  ctrl_arch_->tci_container_->qp_params_->W_delta_qddot_ =
      Eigen::VectorXd::Constant(6, W_delta_qddot_);
  ctrl_arch_->tci_container_->qp_params_->W_delta_rf_
      << W_delta_rf_left_foot_in_contact_,
      W_delta_rf_right_foot_in_contact_;
  ctrl_arch_->tci_container_->qp_params_->W_xc_ddot_ =
      Eigen::VectorXd::Constant(12, W_xc_ddot_in_contact_);
  ctrl_arch_->tci_container_->qp_params_->W_force_rate_of_change_
      << W_force_rate_of_change_left_foot_,
      W_force_rate_of_change_right_foot_;

  // ctrl_arch_->tci_container_->qp_params_->W_xc_ddot_[5] = 1e8;
  // ctrl_arch_->tci_container_->qp_params_->W_xc_ddot_[11] = 1e8;

  // initialize reaction force tasks
  // smoothly increase the fz in world frame
  Eigen::VectorXd init_reaction_force = Eigen::VectorXd::Zero(6);
  init_reaction_force[5] = kGravity * robot_->GetTotalMass() / 2.;
  Eigen::VectorXd des_reaction_force = Eigen::VectorXd::Zero(6);
  des_reaction_force[5] = kGravity * robot_->GetTotalMass() / 2.;
  ctrl_arch_->lf_force_tm_->InitializeInterpolation(
      init_reaction_force, des_reaction_force, end_time_);
  ctrl_arch_->rf_force_tm_->InitializeInterpolation(
      init_reaction_force, des_reaction_force, end_time_);
}

void DoubleSupportStandUp::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // com & torso ori task update
  ctrl_arch_->floating_base_tm_->UpdateDesired(state_machine_time_);

  // foot task
  // ctrl_arch_->lf_SE3_tm_->UseCurrent();
  // ctrl_arch_->rf_SE3_tm_->UseCurrent();

  //  increase maximum normal reaction force
  ctrl_arch_->lf_max_normal_froce_tm_->UpdateRampToMax(state_machine_time_);
  ctrl_arch_->rf_max_normal_froce_tm_->UpdateRampToMax(state_machine_time_);

  // update force traj manager
  ctrl_arch_->lf_force_tm_->UpdateDesired(state_machine_time_);
  ctrl_arch_->rf_force_tm_->UpdateDesired(state_machine_time_);
}

void DoubleSupportStandUp::LastVisit() {}

bool DoubleSupportStandUp::EndOfState() {
  return (state_machine_time_ > end_time_) ? true : false;
}

StateId DoubleSupportStandUp::GetNextState() {
  return draco_states::kDoubleSupportBalance;
}

void DoubleSupportStandUp::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "standup_duration", end_time_);
    std::string prefix = sp_->b_use_base_height_ ? "base" : "com";
    util::ReadParameter(node, "target_" + prefix + "_height", target_height_);
    sp_->des_com_height_ = target_height_;
    sp_->des_body_height_ = target_height_;
    util::ReadParameter(node, "rf_z_max_interp_duration",
                        rf_z_max_interp_duration_);

    // qp params yaml
    YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
    util::ReadParameter(cfg["wbc"]["qp"], "W_delta_qddot", W_delta_qddot_);
    util::ReadParameter(cfg["wbc"]["qp"], "W_xc_ddot_in_contact",
                        W_xc_ddot_in_contact_);
    util::ReadParameter(cfg["wbc"]["qp"], "W_delta_rf_left_foot_in_contact",
                        W_delta_rf_left_foot_in_contact_);
    util::ReadParameter(cfg["wbc"]["qp"], "W_delta_rf_right_foot_in_contact",
                        W_delta_rf_right_foot_in_contact_);
    // util::ReadParameter(cfg["wbc"]["qp"], "W_force_rate_of_change_left_foot",
    // W_force_rate_of_change_left_foot_);
    // util::ReadParameter(cfg["wbc"]["qp"],
    // "W_force_rate_of_change_right_foot", W_force_rate_of_change_right_foot_);

  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

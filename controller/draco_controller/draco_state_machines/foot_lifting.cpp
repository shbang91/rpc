#include "controller/draco_controller/draco_state_machines/foot_lifting.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "util/util.hpp"

FootLifting::FootLifting(const StateId state_id, PinocchioRobotSystem *robot,
                         DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch),
      b_static_walking_trigger_(false) {
  util::PrettyConstructor(2, "FootLifting");

  sp_ = DracoStateProvider::GetStateProvider();
}

void FootLifting::FirstVisit() {
  state_machine_start_time_ = sp_->current_time_;

  // saving nominal foot pose for foot impedance control
  sp_->nominal_right_foot_iso_ =
      robot_->GetLinkIsometry(draco_link::r_foot_contact);
  sp_->nominal_left_foot_iso_ =
      robot_->GetLinkIsometry(draco_link::l_foot_contact);

  if (sp_->stance_foot_ == draco_link::l_foot_contact) {
    std::cout << "draco_states::kRFootLifting" << std::endl;

    sp_->b_rf_contact_ = false;
    sp_->b_lf_contact_ = true;

    Eigen::Isometry3d init_des_foot_iso;
    init_des_foot_iso.translation() =
        // ctrl_arch_->tci_container_->task_map_["rf_pos_task"]->DesiredPos();
        ctrl_arch_->tci_container_->task_map_["rf_pos_task"]->CurrentPos();
    // sp_->nominal_right_foot_iso_.translation();
    Eigen::VectorXd des_rfoot_quat_vec =
        // ctrl_arch_->tci_container_->task_map_["rf_ori_task"]->DesiredPos();
        ctrl_arch_->tci_container_->task_map_["rf_ori_task"]->CurrentPos();
    Eigen::Quaterniond des_rfoot_quat(
        des_rfoot_quat_vec[3], des_rfoot_quat_vec[0], des_rfoot_quat_vec[1],
        des_rfoot_quat_vec[2]);
    init_des_foot_iso.linear() = des_rfoot_quat.normalized().toRotationMatrix();
    // init_des_foot_iso.linear() = sp_->nominal_right_foot_iso_.linear();

    // desired foot pose at apex
    Eigen::Isometry3d target_des_foot_iso;
    target_des_foot_iso.translation() =
        init_des_foot_iso.translation() +
        init_des_foot_iso.linear() * foot_pos_offset_local_;
    // TODO: make this general (getting ori command from yaml)
    target_des_foot_iso.linear() = init_des_foot_iso.linear();

    // initialize swing foot traj
    ctrl_arch_->rf_SE3_tm_->InitializeHalfSwingTrajectory(
        init_des_foot_iso, target_des_foot_iso, end_time_);

  } else if (sp_->stance_foot_ == draco_link::r_foot_contact) {
    std::cout << "draco_states::kLFootLifting" << std::endl;
    sp_->b_lf_contact_ = false;
    sp_->b_rf_contact_ = true;

    Eigen::Isometry3d init_des_foot_iso;
    init_des_foot_iso.translation() =
        // ctrl_arch_->tci_container_->task_map_["lf_pos_task"]->DesiredPos();
        ctrl_arch_->tci_container_->task_map_["lf_pos_task"]->CurrentPos();
    // sp_->nominal_left_foot_iso_.translation();
    Eigen::VectorXd des_lfoot_quat_vec =
        // ctrl_arch_->tci_container_->task_map_["lf_ori_task"]->DesiredPos();
        ctrl_arch_->tci_container_->task_map_["lf_ori_task"]->CurrentPos();
    Eigen::Quaterniond des_lfoot_quat(
        des_lfoot_quat_vec[3], des_lfoot_quat_vec[0], des_lfoot_quat_vec[1],
        des_lfoot_quat_vec[2]);
    init_des_foot_iso.linear() = des_lfoot_quat.normalized().toRotationMatrix();
    // init_des_foot_iso.linear() = sp_->nominal_left_foot_iso_.linear();

    // desired foot pose at apex
    Eigen::Isometry3d target_des_foot_iso;
    target_des_foot_iso.translation() =
        init_des_foot_iso.translation() +
        init_des_foot_iso.linear() * foot_pos_offset_local_;
    // TODO: make this general (getting ori command from yaml)
    target_des_foot_iso.linear() = init_des_foot_iso.linear();

    // initialize swing foot traj
    ctrl_arch_->lf_SE3_tm_->InitializeHalfSwingTrajectory(
        init_des_foot_iso, target_des_foot_iso, end_time_);

  } else {
    assert(false);
  }
}

void FootLifting::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  if (sp_->stance_foot_ == draco_link::l_foot_contact) {
    // rfoot swing
    ctrl_arch_->rf_SE3_tm_->UpdateHalfSwingDesired(state_machine_time_);
    // ctrl_arch_->lf_SE3_tm_->UseCurrent();
    ctrl_arch_->lf_SE3_tm_->UseNominal(sp_->nominal_left_foot_iso_);
  } else if (sp_->stance_foot_ == draco_link::r_foot_contact) {
    // lfoot swing
    ctrl_arch_->lf_SE3_tm_->UpdateHalfSwingDesired(state_machine_time_);
    // ctrl_arch_->rf_SE3_tm_->UseCurrent();
    ctrl_arch_->rf_SE3_tm_->UseNominal(sp_->nominal_right_foot_iso_);
  } else {
    assert(false);
  }
}

void FootLifting::LastVisit() { b_static_walking_trigger_ = false; }

bool FootLifting::EndOfState() {
  return (state_machine_time_ > end_time_ && b_static_walking_trigger_) ? true
                                                                        : false;
}

StateId FootLifting::GetNextState() {
  if (state_id_ == draco_states::kLFootLifting)
    return draco_states::kLFootLanding;
  if (state_id_ == draco_states::kRFootLifting)
    return draco_states::kRFootLanding;
}

void FootLifting::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "moving_duration", end_time_);
    util::ReadParameter(node, "foot_offset_local", foot_pos_offset_local_);
  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

#include "controller/draco_controller/draco_state_machines/double_support_swaying.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_task/draco_com_xy_task.hpp"
#include "controller/draco_controller/draco_task/draco_com_z_task.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/reaction_force_trajectory_manager.hpp"

DoubleSupportSwaying::DoubleSupportSwaying(const StateId state_id,
                                           PinocchioRobotSystem *robot,
                                           DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch),
      amp_(Eigen::Vector3d::Zero()), freq_(Eigen::Vector3d::Zero()) {
  util::PrettyConstructor(2, "DoubleSupportSwaying");

  sp_ = DracoStateProvider::GetStateProvider();
}

void DoubleSupportSwaying::FirstVisit() {
  std::cout << "draco_states: kDoubleSupportSwaying" << std::endl;
  state_machine_start_time_ = sp_->current_time_;

  // Eigen::VectorXd init_com_pos = robot_->GetRobotComPos();
  // if (sp_->b_use_base_height_)
  // init_com_pos[2] =
  // robot_->GetLinkIsometry(draco_link::torso_com_link).translation()[2];
  Eigen::Vector2d des_com_xy =
      ctrl_arch_->tci_container_->task_map_["com_xy_task"]->DesiredPos();
  double des_com_z =
      ctrl_arch_->tci_container_->task_map_["com_z_task"]->DesiredPos()[0];
  Eigen::Vector3d init_com_pos(des_com_xy[0], des_com_xy[1], des_com_z);

  ctrl_arch_->floating_base_tm_->InitializeSwaying(init_com_pos, amp_, freq_);

  // get desired final RF from each foot to use as starting desired reaction
  // force values
  Eigen::VectorXd des_init_lfoot_rf = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd des_init_rfoot_rf = Eigen::VectorXd::Zero(6);
  des_init_lfoot_rf = ctrl_arch_->lf_force_tm_->GetFinalDesiredRf();
  des_init_rfoot_rf = ctrl_arch_->rf_force_tm_->GetFinalDesiredRf();
  ctrl_arch_->lf_force_tm_->InitializeSwaying(des_init_lfoot_rf, amp_, freq_);
  ctrl_arch_->rf_force_tm_->InitializeSwaying(des_init_rfoot_rf, amp_, freq_);
}

void DoubleSupportSwaying::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // com task
  ctrl_arch_->floating_base_tm_->UpdateDesired(state_machine_time_);

  // foot pose task
  ctrl_arch_->lf_SE3_tm_->UseCurrent();
  ctrl_arch_->rf_SE3_tm_->UseCurrent();

  // update force traj manager
  ctrl_arch_->lf_force_tm_->UpdateDesired(state_machine_time_);
  ctrl_arch_->rf_force_tm_->UpdateDesired(state_machine_time_);
}

bool DoubleSupportSwaying::EndOfState() { return false; }

void DoubleSupportSwaying::LastVisit() {}

StateId DoubleSupportSwaying::GetNextState() {}

void DoubleSupportSwaying::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "amplitude", amp_);
    util::ReadParameter(node, "frequency", freq_);
  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [ " << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

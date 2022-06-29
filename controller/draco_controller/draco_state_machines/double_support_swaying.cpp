#include "controller/draco_controller/draco_state_machines/double_support_swaying.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"

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

  Eigen::VectorXd init_com_pos = robot_->GetRobotComPos();
  ctrl_arch_->floating_base_tm_->InitializeSwaying(init_com_pos, amp_, freq_);
}

void DoubleSupportSwaying::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // com task
  ctrl_arch_->floating_base_tm_->UpdateDesired(state_machine_time_);

  // foot pose task
  ctrl_arch_->lf_SE3_tm_->UseCurrent();
  ctrl_arch_->rf_SE3_tm_->UseCurrent();
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

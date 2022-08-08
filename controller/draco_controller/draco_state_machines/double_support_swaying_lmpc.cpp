#include "controller/draco_controller/draco_state_machines/double_support_swaying_lmpc.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_task/draco_com_task.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"

DoubleSupportSwayingLmpc::DoubleSupportSwayingLmpc(
    const StateId state_id, PinocchioRobotSystem *robot,
    DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch),
      amp_(Eigen::Vector3d::Zero()), freq_(Eigen::Vector3d::Zero()),
      b_use_base_height_(false) {
  util::PrettyConstructor(2, "DoubleSupportSwayingLmpc");

  sp_ = DracoStateProvider::GetStateProvider();

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
  int height_source = util::ReadParameter<int>(cfg["wbc"]["task"]["com_task"],
                                               "com_height_target_source");
  b_use_base_height_ =
      height_source == com_height_target_source::kBaseHeight ? true : false;
}

void DoubleSupportSwayingLmpc::FirstVisit() {
  std::cout << "draco_states: kDoubleSupportSwaying" << std::endl;
  state_machine_start_time_ = sp_->current_time_;

  Eigen::VectorXd init_com_pos = robot_->GetRobotComPos();
  if (b_use_base_height_)
    init_com_pos[2] =
        robot_->GetLinkIsometry(draco_link::torso_com_link).translation()[2];
  ctrl_arch_->floating_base_tm_->InitializeSwaying(init_com_pos, amp_, freq_);

  // TODO: initialize lmpc
}

void DoubleSupportSwayingLmpc::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // TODO:
  // Lmpc initial states setup
  // lmpc ref traj setup
  // solve mpc

  // desired task update
  // 1. com task
  // 2. reaction force task

  // com task
  ctrl_arch_->floating_base_tm_->UpdateDesired(state_machine_time_);

  // foot pose task
  ctrl_arch_->lf_SE3_tm_->UseCurrent();
  ctrl_arch_->rf_SE3_tm_->UseCurrent();
}

bool DoubleSupportSwayingLmpc::EndOfState() { return false; }

void DoubleSupportSwayingLmpc::LastVisit() {}

StateId DoubleSupportSwayingLmpc::GetNextState() {}

void DoubleSupportSwayingLmpc::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "amplitude", amp_);
    util::ReadParameter(node, "frequency", freq_);
  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [ " << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

#include "controller/draco_controller/draco_state_machines/double_support_balance.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"

DoubleSupportBalance::DoubleSupportBalance(const StateId state_id,
                                           PinocchioRobotSystem *robot,
                                           DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch),
      b_com_swaying_(false), b_lmpc_swaying_(false), b_nmpc_swaying_(false),
      b_dcm_walking_(false), b_lmpc_walking_(false), b_nmpc_walking_(false),
      b_static_walking_(false) {
  util::PrettyConstructor(2, "DoubleSupportBalance");

  sp_ = DracoStateProvider::GetStateProvider();
}

void DoubleSupportBalance::FirstVisit() {
  std::cout << "draco_states: kDoubleSupportBalance" << std::endl;
  state_machine_start_time_ = sp_->current_time_;

  // reset flags
  b_com_swaying_ = false;
  b_lmpc_swaying_ = false;
  b_nmpc_swaying_ = false;

  b_dcm_walking_ = false;
  b_lmpc_walking_ = false;
  b_nmpc_walking_ = false;

  b_static_walking_ = false;
}

void DoubleSupportBalance::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // update foot pose task
  ctrl_arch_->lf_SE3_tm_->UseCurrent();
  ctrl_arch_->rf_SE3_tm_->UseCurrent();
}

bool DoubleSupportBalance::EndOfState() {
  // TODO: add dcm walking condition(e.g., remaining footstep)
  return (b_com_swaying_ || b_lmpc_swaying_ || b_nmpc_swaying_ ||
          b_dcm_walking_ || b_lmpc_walking_ || b_nmpc_walking_ ||
          b_static_walking_)
             ? true
             : false;
}

void DoubleSupportBalance::LastVisit() {}

StateId DoubleSupportBalance::GetNextState() {
  if (b_com_swaying_)
    return draco_states::kDoubleSupportSwaying;
  // if (b_lmpc_swaying_)
  // return draco_states::kComSwayingLmpc;
  // if (b_nmpc_swaying_)
  // return draco_states::kComSwayingNmpc;

  // if (b_dcm_walking_) {
  // //which foot (L or R?)
  // if (true) {
  // return draco_states::kLFootContactTransitionStart;
  // else {
  // return draco_states::kRFootContactTransitionStart;
  //}
  //}
  //}

  // if (b_lmpc_walking_)
  // return;
  // if (b_nmpc_walking_)
  // return;

  // if (b_static_walking_) {
  // if (true) {
  // return draco_states::kMoveComToLFoot;
  //} else {
  // return draco_states::kMoveComToRFoot;
  //}
  //}
}

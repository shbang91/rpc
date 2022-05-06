#include "pnc/fixed_draco_pnc/fixed_draco_control_architecture.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_controller.hpp"
//#include "pnc/fixed_draco_pnc/fixed_draco_state_machine/initialize.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"
#include "pnc/robot_system/robot_system.hpp"

FixedDracoControlArchitecture::FixedDracoControlArchitecture(
    RobotSystem *_robot)
    : ControlArchitecture(_robot) {
  robot_ = _robot;

  prev_state_ = FixedDracoState::kInitialize;
  state_ = FixedDracoState::kInitialize;

  b_state_first_visit_ = true;

  controller_ = new FixedDracoController(robot_);

  sp_ = FixedDracoStateProvider::GetStateProvider();

  // state_machines_container_[FixedDracoState::kInitialize] =
  // new Initialize(FixedDracoState::kInitialize, robot_);

  // state_machine_container_[FixedDracoState::kRightFootMove] =
  // new EndEffectorMove(FixedDracoSate::kRightFootMove, robot_);
}

FixedDracoControlArchitecture::~FixedDracoControlArchitecture() {
  // delete state_machines_container_[FixedDracoState::kInitialize];
  // delete controller_
}

void FixedDracoControlArchitecture::GetCommand(void *_command) {
  // finite state machine
  if (b_state_first_visit_) {
    // state_machines_container_[state_]->FirstVisit();
    b_state_first_visit_ = false;
  }
  // state_machines_container_[state_]->OneStep();

  // Controller
  controller_->GetCommand(_command);

  // if (state_machines_container_[state_]->EndOfState()) {
  // state_machines_container_[state_]->LastVisit();
  // prev_state_ = state_;
  // state_ = state_machines_container_[state_]->GetNextState();
  // b_state_first_visit_ = true;
  //}
}

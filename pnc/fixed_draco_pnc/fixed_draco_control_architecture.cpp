#include "pnc/fixed_draco_pnc/fixed_draco_control_architecture.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_controller.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_machines/initialize.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_tci_container.hpp"
#include "pnc/robot_system/robot_system.hpp"
#include "util/util.hpp"

FixedDracoControlArchitecture::FixedDracoControlArchitecture(
    RobotSystem *_robot)
    : ControlArchitecture(_robot) {
  robot_ = _robot;

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/fixed_draco/pnc.yaml");

  prev_state_ = FixedDracoState::kInitialize;
  state_ = FixedDracoState::kInitialize;

  b_state_first_visit_ = true;

  tci_container_ = new FixedDracoTCIContainer(robot_);

  controller_ = new FixedDracoController(tci_container_, robot_);

  sp_ = FixedDracoStateProvider::GetStateProvider();

  state_machines_container_[FixedDracoState::kInitialize] =
      new Initialize(FixedDracoState::kInitialize, this, robot_);
  (static_cast<Initialize *>(
       state_machines_container_[FixedDracoState::kInitialize]))
      ->duration_ = util::ReadParameter<double>(cfg["behavior"], "duration");
  YAML::Node node = cfg["behavior"]["ini_joint_pos"];
  std::map<std::string, double> jpos_map;
  for (const auto &k : node) {
    jpos_map[k.first.as<std::string>()] = k.second.as<double>();
  }
  Eigen::VectorXd desired_ini_jpos = robot_->MapToEigenVector(jpos_map);
  (static_cast<Initialize *>(
       state_machines_container_[FixedDracoState::kInitialize]))
      ->des_jpos_ = desired_ini_jpos;

  // state_machine_container_[FixedDracoState::kRightFootMove] =
  // new EndEffectorMove(FixedDracoSate::kRightFootMove, robot_);
}

FixedDracoControlArchitecture::~FixedDracoControlArchitecture() {
  delete state_machines_container_[FixedDracoState::kInitialize];
  delete tci_container_;
  delete controller_;
}

void FixedDracoControlArchitecture::GetCommand(void *_command) {
  // finite state machine
  if (b_state_first_visit_) {
    state_machines_container_[state_]->FirstVisit();
    b_state_first_visit_ = false;
  }
  state_machines_container_[state_]->OneStep();

  // Controller
  controller_->GetCommand(_command);

  if (state_machines_container_[state_]->EndOfState()) {
    state_machines_container_[state_]->LastVisit();
    prev_state_ = state_;
    state_ = state_machines_container_[state_]->GetNextState();
    b_state_first_visit_ = true;
  }
}

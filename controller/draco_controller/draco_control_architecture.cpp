#include "controller/draco_controller/draco_control_architecture.hpp"
#include "util/util.hpp"

DracoControlArchitecture::DracoControlArchitecture(PinocchioRobotSystem *robot)
    : ControlArchitecture(robot) {
  util::PrettyConstructor(1, "DracoControlArchitecture");
  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

  bool b_sim = util::ReadParameter<bool>(cfg, "b_sim");
  prev_state_ = (b_sim) ? DracoStates::kStandUp : DracoStates::kInitialize;
  state_ = (b_sim) ? DracoStates::kStandUp : DracoStates::kInitialize;

  // tci_container_ = new DracoTCIContainer(robot_);
  //  controller_ = new DracoController(tci_container_, robot_);

  // state_machine_container_[DracoStates::kInitialize] =
  // new Initialize(DracoStates::kInitialize, robot_);
  // state_machine_container_[DracoStates::kStandUp] =
  // new StandUp(DracoStates::kStandUp, robot_);
  // state_machine_container_[DracoStates::kBalance] =
  // new Balance(DracoStates::kBalance, robot_);
}

DracoControlArchitecture::~DracoControlArchitecture() {
  // delete tci_container_;
  // delete controller_;
  // delete state_machine_container_[DracoStates::kInitialize];
  // delete state_machine_container_[DracoStates::kStandUp];
  // delete state_machine_container_[DracoStates::kBalance];
}

void DracoControlArchitecture::GetCommand(void *command) {
  if (b_state_first_visit_) {
    state_machine_container_[state_]->FirstVisit();
    b_state_first_visit_ = false;
  }

  if (!state_machine_container_[state_]->EndOfState()) {
    state_machine_container_[state_]->OneStep();
    // controller_->GetCommand(command);
  } else {
    state_machine_container_[state_]->LastVisit();
    prev_state_ = state_;
    state_ = state_machine_container_[state_]->GetNextState();
    b_state_first_visit_ = true;
  }
}

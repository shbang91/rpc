#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_controller.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_stand_up.hpp"
#include "controller/draco_controller/draco_state_machines/initialize.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/upper_body_trajectory_manager.hpp"
#include "util/util.hpp"

DracoControlArchitecture::DracoControlArchitecture(PinocchioRobotSystem *robot)
    : ControlArchitecture(robot) {
  util::PrettyConstructor(1, "DracoControlArchitecture");

  // set starting state
  cfg_ = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
  bool b_sim = util::ReadParameter<bool>(cfg_, "b_sim");
  prev_state_ =
      (b_sim) ? draco_states::kDoubleSupportStandUp : draco_states::kInitialize;
  state_ =
      (b_sim) ? draco_states::kDoubleSupportStandUp : draco_states::kInitialize;
  b_state_first_visit_ = true;

  // initialize controller
  tci_container_ = new DracoTCIContainer(robot_);
  controller_ = new DracoController(tci_container_, robot_);

  // initialize manager
  floating_base_tm_ = new FloatingBaseTrajectoryManager(
      tci_container_->com_task_, tci_container_->torso_ori_task_, robot_);
  upper_body_tm_ =
      new UpperBodyTrajetoryManager(tci_container_->upper_body_task_, robot_);

  // initialize states
  state_machine_container_[draco_states::kInitialize] =
      new Initialize(draco_states::kInitialize, robot_, this);
  state_machine_container_[draco_states::kDoubleSupportStandUp] =
      new DoubleSupportStandUp(draco_states::kDoubleSupportStandUp, robot_,
                               this);
  // state_machine_container_[draco_states::kDoubleSupportBalance] =
  // new Balance(draco_states::kDoubleSupportBalance, robot_);

  sp_ = DracoStateProvider::GetStateProvider();

  this->_InitializeParameters();
}

DracoControlArchitecture::~DracoControlArchitecture() {
  delete tci_container_;
  delete controller_;
  delete state_machine_container_[draco_states::kInitialize];
  delete state_machine_container_[draco_states::kDoubleSupportStandUp];
  // delete state_machine_container_[draco_states::kDoubleSupportBalance];
}

void DracoControlArchitecture::GetCommand(void *command) {
  if (b_state_first_visit_) {
    state_machine_container_[state_]->FirstVisit();
    b_state_first_visit_ = false;
  }

  if (!state_machine_container_[state_]->EndOfState()) {
    state_machine_container_[state_]->OneStep();
    // state independent upper body traj setting
    upper_body_tm_->UseNominalUpperBodyJointPos(sp_->nominal_jpos_);
    controller_->GetCommand(command);
  } else {
    state_machine_container_[state_]->LastVisit();
    prev_state_ = state_;
    state_ = state_machine_container_[state_]->GetNextState();
    b_state_first_visit_ = true;
  }
}

void DracoControlArchitecture::_InitializeParameters() {
  // state machine initialization
  state_machine_container_[draco_states::kInitialize]->InitializeParameters(
      cfg_["state_machine"]["initialize"]);
  state_machine_container_[draco_states::kDoubleSupportStandUp]
      ->InitializeParameters(cfg_["state_machine"]["stand_up"]);
}

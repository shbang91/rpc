#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_controller.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_stand_up.hpp"
#include "controller/draco_controller/draco_state_machines/initialize.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
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

  //=============================================================
  // initialize task, contact, controller
  //=============================================================
  tci_container_ = new DracoTCIContainer(robot_);
  controller_ = new DracoController(tci_container_, robot_);

  //=============================================================
  // trajectory Managers
  //=============================================================
  //  initialize kinematics manager
  upper_body_tm_ =
      new UpperBodyTrajetoryManager(tci_container_->upper_body_task_, robot_);
  floating_base_tm_ = new FloatingBaseTrajectoryManager(
      tci_container_->com_task_, tci_container_->torso_ori_task_, robot_);
  lf_SE3_tm_ = new EndEffectorTrajectoryManager(
      tci_container_->lf_pos_task_, tci_container_->lf_ori_task_, robot_);
  rf_SE3_tm_ = new EndEffectorTrajectoryManager(
      tci_container_->rf_pos_task_, tci_container_->rf_ori_task_, robot_);

  // initialize dynamics manager

  double max_rf_z =
      b_sim
          ? util::ReadParameter<double>(cfg_["wbc"]["contact"], "max_rf_z")
          : util::ReadParameter<double>(cfg_["wbc"]["contact"], "exp_max_rf_z");
  lf_max_normal_froce_tm_ = new MaxNormalForceTrajectoryManager(
      tci_container_->lf_contact_, max_rf_z);
  rf_max_normal_froce_tm_ = new MaxNormalForceTrajectoryManager(
      tci_container_->rf_contact_, max_rf_z);

  //=============================================================
  // initialize state machines
  //=============================================================
  state_machine_container_[draco_states::kInitialize] =
      new Initialize(draco_states::kInitialize, robot_, this);
  state_machine_container_[draco_states::kDoubleSupportStandUp] =
      new DoubleSupportStandUp(draco_states::kDoubleSupportStandUp, robot_,
                               this);
  // state_machine_container_[draco_states::kDoubleSupportBalance] =
  // new DoubleSupportBalance(draco_states::kDoubleSupportBalance,
  // robot_);

  sp_ = DracoStateProvider::GetStateProvider();

  this->_InitializeParameters();
}

DracoControlArchitecture::~DracoControlArchitecture() {
  delete tci_container_;
  delete controller_;
  delete upper_body_tm_;
  delete floating_base_tm_;
  delete lf_SE3_tm_;
  delete rf_SE3_tm_;
  delete lf_max_normal_froce_tm_;
  delete rf_max_normal_froce_tm_;
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
  state_machine_container_[draco_states::kInitialize]->SetParameters(
      cfg_["state_machine"]["initialize"]);
  state_machine_container_[draco_states::kDoubleSupportStandUp]->SetParameters(
      cfg_["state_machine"]["stand_up"]);
}

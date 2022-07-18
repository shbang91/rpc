#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_controller.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_machines/contact_transition_start.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_balance.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_stand_up.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_swaying.hpp"
#include "controller/draco_controller/draco_state_machines/initialize.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/mpc_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "controller/whole_body_controller/managers/upper_body_trajectory_manager.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"
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
  // initialize task, contact, controller, planner
  //=============================================================
  tci_container_ = new DracoTCIContainer(robot_);
  controller_ = new DracoController(tci_container_, robot_);

  dcm_planner_ = new DCMPlanner();

  //=============================================================
  // trajectory Managers
  //=============================================================
  //  initialize kinematics manager
  upper_body_tm_ =
      new UpperBodyTrajetoryManager(tci_container_->upper_body_task_, robot_);
  floating_base_tm_ = new FloatingBaseTrajectoryManager(
      tci_container_->com_task_, tci_container_->torso_ori_task_, robot_);
  dcm_tm_ = new DCMTrajectoryManager(
      dcm_planner_, tci_container_->com_task_, tci_container_->torso_ori_task_,
      robot_, draco_link::l_foot_contact, draco_link::r_foot_contact);

  mpc_tm_ = new MPCTrajectoryManager(tci_container_->com_task_, tci_container_->torso_ori_task_,
                                     robot_, draco_link::l_foot_contact, draco_link::r_foot_contact);
  lf_SE3_tm_ = new EndEffectorTrajectoryManager(
      tci_container_->lf_pos_task_, tci_container_->lf_ori_task_, robot_);
  rf_SE3_tm_ = new EndEffectorTrajectoryManager(
      tci_container_->rf_pos_task_, tci_container_->rf_ori_task_, robot_);

  Eigen::VectorXd pos_weight_in_contact =
      b_sim ? util::ReadParameter<Eigen::VectorXd>(
                  cfg_["wbc"]["task"]["foot_pos_task"], "weight")
            : util::ReadParameter<Eigen::VectorXd>(
                  cfg_["wbc"]["task"]["foot_pos_task"], "exp_weight");
  Eigen::VectorXd ori_weight_in_contact =
      b_sim ? util::ReadParameter<Eigen::VectorXd>(
                  cfg_["wbc"]["task"]["foot_ori_task"], "weight")
            : util::ReadParameter<Eigen::VectorXd>(
                  cfg_["wbc"]["task"]["foot_ori_task"], "exp_weight");
  Eigen::VectorXd pos_weight_in_swing =
      b_sim ? util::ReadParameter<Eigen::VectorXd>(
                  cfg_["wbc"]["task"]["foot_pos_task"], "weight_at_swing")
            : util::ReadParameter<Eigen::VectorXd>(
                  cfg_["wbc"]["task"]["foot_pos_task"], "exp_weight_at_swing");
  Eigen::VectorXd ori_weight_in_swing =
      b_sim ? util::ReadParameter<Eigen::VectorXd>(
                  cfg_["wbc"]["task"]["foot_ori_task"], "weight_at_swing")
            : util::ReadParameter<Eigen::VectorXd>(
                  cfg_["wbc"]["task"]["foot_ori_task"], "exp_weight_at_swing");
  lf_pos_hm_ = new TaskHierarchyManager(
      tci_container_->lf_pos_task_, pos_weight_in_contact, pos_weight_in_swing);
  lf_ori_hm_ = new TaskHierarchyManager(
      tci_container_->lf_ori_task_, ori_weight_in_contact, ori_weight_in_swing);
  rf_pos_hm_ = new TaskHierarchyManager(
      tci_container_->rf_pos_task_, pos_weight_in_contact, pos_weight_in_swing);
  rf_ori_hm_ = new TaskHierarchyManager(
      tci_container_->rf_ori_task_, pos_weight_in_contact, pos_weight_in_swing);

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
  state_machine_container_[draco_states::kDoubleSupportBalance] =
      new DoubleSupportBalance(draco_states::kDoubleSupportBalance, robot_,
                               this);
  state_machine_container_[draco_states::kDoubleSupportSwaying] =
      new DoubleSupportSwaying(draco_states::kDoubleSupportSwaying, robot_,
                               this);
  state_machine_container_[draco_states::kLFContactTransitionStart] =
      new ContactTransitionStart(draco_states::kLFContactTransitionStart,
                                 robot_, this);

  state_machine_container_[draco_states::kRFContactTransitionStart] =
      new ContactTransitionStart(draco_states::kRFContactTransitionStart,
                                 robot_, this);

  sp_ = DracoStateProvider::GetStateProvider();

  this->_InitializeParameters();

  // ZMQ Sockets (MPC comms)
  context_pub_ = new zmq::context_t(1);
  publisher_ = new zmq::socket_t(*context_pub_, ZMQ_PUB);
  publisher_->bind("tcp://127.0.0.2:5557");
  footstep_list_index_ = -2;

  context_sub_ = new zmq::context_t(1);
  subscriber_ = new zmq::socket_t(*context_sub_, ZMQ_SUB);
  subscriber_->connect("tcp://127.0.0.3:5557");
  subscriber_->setsockopt(ZMQ_SUBSCRIBE, NULL, 0);
}

DracoControlArchitecture::~DracoControlArchitecture() {
  delete tci_container_;
  delete controller_;
  delete dcm_planner_;

  // tm
  delete upper_body_tm_;
  delete floating_base_tm_;
  delete lf_SE3_tm_;
  delete rf_SE3_tm_;
  delete lf_max_normal_froce_tm_;
  delete rf_max_normal_froce_tm_;
  delete dcm_tm_;

  // hm
  delete lf_pos_hm_;
  delete lf_ori_hm_;
  delete rf_pos_hm_;
  delete rf_ori_hm_;

  // state machines
  delete state_machine_container_[draco_states::kInitialize];
  delete state_machine_container_[draco_states::kDoubleSupportStandUp];
  delete state_machine_container_[draco_states::kDoubleSupportBalance];
  delete state_machine_container_[draco_states::kDoubleSupportSwaying];
  delete state_machine_container_[draco_states::kLFContactTransitionStart];
  delete state_machine_container_[draco_states::kRFContactTransitionStart];
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
  state_machine_container_[draco_states::kDoubleSupportSwaying]->SetParameters(
      cfg_["state_machine"]["com_swaying"]);

  // dcm planner params initialization
  dcm_tm_->InitializeParameters(cfg_["dcm_walking"]);
}

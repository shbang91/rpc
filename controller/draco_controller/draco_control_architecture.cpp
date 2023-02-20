#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_controller.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_machines/contact_transition_end.hpp"
#include "controller/draco_controller/draco_state_machines/contact_transition_start.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_balance.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_stand_up.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_swaying.hpp"
#include "controller/draco_controller/draco_state_machines/single_support_swing.hpp"
//#include
//"controller/draco_controller/draco_state_machines/double_support_swaying_lmpc.hpp"
#include "controller/draco_controller/draco_state_machines/initialize.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
//#include "controller/model_predictive_controller/lmpc/lmpc_handler.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/reaction_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "controller/whole_body_controller/managers/upper_body_trajectory_manager.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"
#include "util/util.hpp"

DracoControlArchitecture::DracoControlArchitecture(PinocchioRobotSystem *robot)
    : ControlArchitecture(robot) {
  util::PrettyConstructor(1, "DracoControlArchitecture");

  sp_ = DracoStateProvider::GetStateProvider();

  try {
    cfg_ = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }

  bool b_sim = util::ReadParameter<bool>(cfg_, "b_sim");

  // set starting state
  prev_state_ =
      b_sim ? draco_states::kDoubleSupportStandUp : draco_states::kInitialize;
  state_ =
      b_sim ? draco_states::kDoubleSupportStandUp : draco_states::kInitialize;

  std::string prefix = b_sim ? "sim" : "exp";

  //=============================================================
  // initialize task, contact, controller, planner
  //=============================================================
  tci_container_ = new DracoTCIContainer(robot_);
  controller_ = new DracoController(tci_container_, robot_);

  dcm_planner_ = new DCMPlanner();

  // mpc handler
  // lmpc_handler_ = new LMPCHandler(
  // dcm_planner_, robot_, tci_container_->com_task_,
  // tci_container_->torso_ori_task_, tci_container_->lf_reaction_force_task_,
  // tci_container_->rf_reaction_force_task_, draco_link::l_foot_contact,
  // draco_link::r_foot_contact);

  //=============================================================
  // trajectory Managers
  //=============================================================
  //  initialize kinematics manager
  upper_body_tm_ = new UpperBodyTrajetoryManager(
      tci_container_->task_map_["upper_body_task"], robot_);
  floating_base_tm_ = new FloatingBaseTrajectoryManager(
      tci_container_->task_map_["com_xy_task"],
      tci_container_->task_map_["com_z_task"],
      tci_container_->task_map_["torso_ori_task"], robot_);
  dcm_tm_ = new DCMTrajectoryManager(
      dcm_planner_, tci_container_->task_map_["com_xy_task"],
      tci_container_->task_map_["com_z_task"],
      tci_container_->task_map_["torso_ori_task"], robot_,
      draco_link::l_foot_contact, draco_link::r_foot_contact,
      sp_->b_use_base_height_);
  lf_SE3_tm_ = new EndEffectorTrajectoryManager(
      tci_container_->task_map_["lf_pos_task"],
      tci_container_->task_map_["lf_ori_task"], robot_);
  rf_SE3_tm_ = new EndEffectorTrajectoryManager(
      tci_container_->task_map_["rf_pos_task"],
      tci_container_->task_map_["rf_ori_task"], robot_);

  Eigen::VectorXd weight_at_contact, weight_at_swing;
  try {
    util::ReadParameter(cfg_["wbc"]["task"]["foot_pos_task"],
                        prefix + "_weight", weight_at_contact);
    util::ReadParameter(cfg_["wbc"]["task"]["foot_pos_task"],
                        prefix + "_weight_at_swing", weight_at_swing);
  } catch (const std::runtime_error &ex) {
    std::cerr << "Error reading parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  lf_pos_hm_ =
      new TaskHierarchyManager(tci_container_->task_map_["lf_pos_task"],
                               weight_at_contact, weight_at_swing);
  rf_pos_hm_ =
      new TaskHierarchyManager(tci_container_->task_map_["rf_pos_task"],
                               weight_at_contact, weight_at_swing);

  try {
    util::ReadParameter(cfg_["wbc"]["task"]["foot_ori_task"],
                        prefix + "_weight", weight_at_contact);
    util::ReadParameter(cfg_["wbc"]["task"]["foot_ori_task"],
                        prefix + "_weight_at_swing", weight_at_swing);
  } catch (const std::runtime_error &ex) {
    std::cerr << "Error reading parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  lf_ori_hm_ =
      new TaskHierarchyManager(tci_container_->task_map_["lf_ori_task"],
                               weight_at_contact, weight_at_swing);
  rf_ori_hm_ =
      new TaskHierarchyManager(tci_container_->task_map_["rf_ori_task"],
                               weight_at_contact, weight_at_swing);

  // initialize dynamics manager
  double max_rf_z;
  util::ReadParameter(cfg_["wbc"]["contact"], prefix + "_max_rf_z", max_rf_z);
  lf_max_normal_froce_tm_ = new MaxNormalForceTrajectoryManager(
      tci_container_->contact_map_["lf_contact"], max_rf_z);
  rf_max_normal_froce_tm_ = new MaxNormalForceTrajectoryManager(
      tci_container_->contact_map_["rf_contact"], max_rf_z);

  lf_force_tm_ = new ForceTrajectoryManager(
      tci_container_->force_task_map_["lf_force_task"], robot_);
  rf_force_tm_ = new ForceTrajectoryManager(
      tci_container_->force_task_map_["rf_force_task"], robot_);

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
  // state_machine_container_[draco_states::kDoubleSupportSwayingLmpc] =
  // new DoubleSupportSwayingLmpc(draco_states::kDoubleSupportSwayingLmpc,
  // robot_, this);
  state_machine_container_[draco_states::kLFContactTransitionStart] =
      new ContactTransitionStart(draco_states::kLFContactTransitionStart,
                                 robot_, this);
  state_machine_container_[draco_states::kLFContactTransitionEnd] =
      new ContactTransitionEnd(draco_states::kLFContactTransitionEnd, robot_,
                               this);
  state_machine_container_[draco_states::kLFSingleSupportSwing] =
      new SingleSupportSwing(draco_states::kLFSingleSupportSwing, robot_, this);

  state_machine_container_[draco_states::kRFContactTransitionStart] =
      new ContactTransitionStart(draco_states::kRFContactTransitionStart,
                                 robot_, this);
  state_machine_container_[draco_states::kRFContactTransitionEnd] =
      new ContactTransitionEnd(draco_states::kRFContactTransitionEnd, robot_,
                               this);
  state_machine_container_[draco_states::kRFSingleSupportSwing] =
      new SingleSupportSwing(draco_states::kRFSingleSupportSwing, robot_, this);

  this->_InitializeParameters();
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
  delete lf_force_tm_;
  delete rf_force_tm_;

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
  // delete state_machine_container_[draco_states::kDoubleSupportSwayingLmpc];
  delete state_machine_container_[draco_states::kLFContactTransitionStart];
  delete state_machine_container_[draco_states::kRFContactTransitionStart];
  delete state_machine_container_[draco_states::kLFContactTransitionEnd];
  delete state_machine_container_[draco_states::kRFContactTransitionEnd];
  delete state_machine_container_[draco_states::kLFSingleSupportSwing];
  delete state_machine_container_[draco_states::kRFSingleSupportSwing];
}

void DracoControlArchitecture::GetCommand(void *command) {
  if (b_state_first_visit_) {
    state_machine_container_[state_]->FirstVisit();
    b_state_first_visit_ = false;
  }

  state_machine_container_[state_]->OneStep();
  upper_body_tm_->UseNominalUpperBodyJointPos(
      sp_->nominal_jpos_);          // state independent upper body traj setting
  controller_->GetCommand(command); // get control command

  if (state_machine_container_[state_]->EndOfState()) {
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
  state_machine_container_[draco_states::kLFSingleSupportSwing]->SetParameters(
      cfg_["state_machine"]["single_support_swing"]);
  state_machine_container_[draco_states::kRFSingleSupportSwing]->SetParameters(
      cfg_["state_machine"]["single_support_swing"]);
  state_machine_container_[draco_states::kDoubleSupportSwaying]->SetParameters(
      cfg_["state_machine"]["com_swaying"]);

  // state_machine_container_[draco_states::kDoubleSupportSwayingLmpc]
  //->SetParameters(cfg_["state_machine"]["lmpc_com_swaying"]);

  // dcm planner params initialization
  dcm_tm_->InitializeParameters(cfg_["dcm_walking"]);
}

#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_controller.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_machines/contact_transition_end.hpp"
#include "controller/draco_controller/draco_state_machines/contact_transition_start.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_balance.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_stand_up.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_swaying.hpp"
#include "controller/draco_controller/draco_state_machines/initialize.hpp"
#include "controller/draco_controller/draco_state_machines/single_support_swing.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/hand_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/reaction_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "controller/whole_body_controller/managers/upper_body_trajectory_manager.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"
#include "util/util.hpp"

#if B_USE_TELEOP
#include "controller/draco_controller/draco_state_machines/teleop_manipulation.hpp"
#endif

#if B_USE_FOXGLOVE
#include "UI/foxglove/client/parameter_subscriber.hpp"
#include "UI/foxglove/client/fox_advertiser.hpp"
#endif

DracoControlArchitecture::DracoControlArchitecture(PinocchioRobotSystem *robot,
                                                   const YAML::Node &cfg)
    : ControlArchitecture(robot) {
  util::PrettyConstructor(1, "DracoControlArchitecture");

  sp_ = DracoStateProvider::GetStateProvider();

  // set starting state
  std::string test_env_name = util::ReadParameter<std::string>(cfg, "env");
  if (test_env_name == "pybullet") {
    prev_loco_state_ = draco_states::kDoubleSupportStandUp;
    loco_state_ = draco_states::kDoubleSupportStandUp;
  } else {
    // mujoco & hw
    prev_loco_state_ = draco_states::kInitialize;
    loco_state_ = draco_states::kInitialize;
  }
  prev_manip_state_ = draco_states::kTeleopManipulation;
  manip_state_ = draco_states::kTeleopManipulation;

  //=============================================================
  // initialize task, contact, controller, planner
  //=============================================================
  tci_container_ = new DracoTCIContainer(robot_, cfg);
  controller_ = new DracoController(tci_container_, robot_, cfg);

  dcm_planner_ = new DCMPlanner();

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
  dcm_tm_->InitializeParameters(cfg["dcm_walking"]);

  lf_SE3_tm_ = new EndEffectorTrajectoryManager(
      tci_container_->task_map_["lf_pos_task"],
      tci_container_->task_map_["lf_ori_task"], robot_);
  rf_SE3_tm_ = new EndEffectorTrajectoryManager(
      tci_container_->task_map_["rf_pos_task"],
      tci_container_->task_map_["rf_ori_task"], robot_);
  lh_SE3_tm_ = new HandTrajectoryManager(
      tci_container_->task_map_["lh_pos_task"],
      tci_container_->task_map_["lh_ori_task"], robot_);
  rh_SE3_tm_ = new HandTrajectoryManager(
      tci_container_->task_map_["rh_pos_task"],
      tci_container_->task_map_["rh_ori_task"], robot_);

  //=============================================================
  // foot task hierarchy manager
  //=============================================================
  Eigen::VectorXd weight_at_contact, weight_at_swing;
  try {
    util::ReadParameter(cfg["wbc"]["task"]["foot_pos_task"], "weight",
                        weight_at_contact);
    util::ReadParameter(cfg["wbc"]["task"]["foot_pos_task"], "weight_at_swing",
                        weight_at_swing);
  } catch (const std::runtime_error &ex) {
    std::cerr << "Error reading foot pos task parameter [" << ex.what()
              << "] at file: [" << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  lf_pos_hm_ =
      new TaskHierarchyManager(tci_container_->task_map_["lf_pos_task"],
                               weight_at_contact, weight_at_swing);
  rf_pos_hm_ =
      new TaskHierarchyManager(tci_container_->task_map_["rf_pos_task"],
                               weight_at_contact, weight_at_swing);

  try {
    util::ReadParameter(cfg["wbc"]["task"]["foot_ori_task"], "weight",
                        weight_at_contact);
    util::ReadParameter(cfg["wbc"]["task"]["foot_ori_task"], "weight_at_swing",
                        weight_at_swing);
  } catch (const std::runtime_error &ex) {
    std::cerr << "Error reading foot ori task parameter [" << ex.what()
              << "] at file: [" << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  lf_ori_hm_ =
      new TaskHierarchyManager(tci_container_->task_map_["lf_ori_task"],
                               weight_at_contact, weight_at_swing);
  rf_ori_hm_ =
      new TaskHierarchyManager(tci_container_->task_map_["rf_ori_task"],
                               weight_at_contact, weight_at_swing);

  //=============================================================
  // hand task hierarchy manager
  //=============================================================
  Eigen::VectorXd weight_at_initial, weight_at_teleop_triggered;
  try {
    util::ReadParameter(cfg["wbc"]["task"]["hand_pos_task"], "weight",
                        weight_at_initial);
    util::ReadParameter(cfg["wbc"]["task"]["hand_pos_task"], "weight_at_teleop",
                        weight_at_teleop_triggered);
  } catch (const std::runtime_error &ex) {
    std::cerr << "Error reading hand pos task parameter [" << ex.what()
              << "] at file: [" << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  lh_pos_hm_ =
      new TaskHierarchyManager(tci_container_->task_map_["lh_pos_task"],
                               weight_at_teleop_triggered, weight_at_initial);
  rh_pos_hm_ =
      new TaskHierarchyManager(tci_container_->task_map_["rh_pos_task"],
                               weight_at_teleop_triggered, weight_at_initial);
  try {
    util::ReadParameter(cfg["wbc"]["task"]["hand_ori_task"], "weight",
                        weight_at_initial);
    util::ReadParameter(cfg["wbc"]["task"]["hand_ori_task"], "weight_at_teleop",
                        weight_at_teleop_triggered);
  } catch (const std::runtime_error &ex) {
    std::cerr << "Error reading hand ori task parameter [" << ex.what()
              << "] at file: [" << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
  lh_ori_hm_ =
      new TaskHierarchyManager(tci_container_->task_map_["lh_ori_task"],
                               weight_at_teleop_triggered, weight_at_initial);
  rh_ori_hm_ =
      new TaskHierarchyManager(tci_container_->task_map_["rh_ori_task"],
                               weight_at_teleop_triggered, weight_at_initial);

  // initialize dynamics manager
  double max_rf_z;
  util::ReadParameter(cfg["wbc"]["contact"], "max_rf_z", max_rf_z);
  lf_max_normal_froce_tm_ = new MaxNormalForceTrajectoryManager(
      tci_container_->contact_map_["lf_contact"], max_rf_z);
  rf_max_normal_froce_tm_ = new MaxNormalForceTrajectoryManager(
      tci_container_->contact_map_["rf_contact"], max_rf_z);

  lf_force_tm_ = new ForceTrajectoryManager(
      tci_container_->force_task_map_["lf_force_task"], robot_);
  rf_force_tm_ = new ForceTrajectoryManager(
      tci_container_->force_task_map_["rf_force_task"], robot_);

  //=============================================================
  // attach Foxglove Clients to control parameters
  //=============================================================
#if B_USE_FOXGLOVE
  param_map_int_ = {{{"n_steps", dcm_tm_->GetNumSteps()}}};
  param_map_double_ = {{{"t_ss", dcm_planner_->GetTssPtr()},
                        {"t_ds", dcm_planner_->GetTdsPtr()}}};
  param_map_hm_ = {{{"lf_pos_task", lf_pos_hm_},
                    {"rf_pos_task", rf_pos_hm_},
                    {"lf_ori_task", lf_ori_hm_},
                    {"rf_ori_task", rf_ori_hm_}}};
  param_subscriber_ =
      new FoxgloveParameterSubscriber(param_map_int_, param_map_double_,
                                      tci_container_->task_map_, param_map_hm_);
  topic_subscriber_ =
      new FoxgloveTopicSubscriber();
#endif

  //=============================================================
  // initialize state machines
  //=============================================================
  // Locomotion
  locomotion_state_machine_container_[draco_states::kInitialize] =
      new Initialize(draco_states::kInitialize, robot_, this);
  locomotion_state_machine_container_[draco_states::kInitialize]->SetParameters(
      cfg);

  locomotion_state_machine_container_[draco_states::kDoubleSupportStandUp] =
      new DoubleSupportStandUp(draco_states::kDoubleSupportStandUp, robot_,
                               this);
  locomotion_state_machine_container_[draco_states::kDoubleSupportStandUp]
      ->SetParameters(cfg);

  locomotion_state_machine_container_[draco_states::kDoubleSupportBalance] =
      new DoubleSupportBalance(draco_states::kDoubleSupportBalance, robot_,
                               this);
  locomotion_state_machine_container_[draco_states::kDoubleSupportBalance]
      ->SetParameters(cfg);

  locomotion_state_machine_container_[draco_states::kDoubleSupportSwaying] =
      new DoubleSupportSwaying(draco_states::kDoubleSupportSwaying, robot_,
                               this);
  locomotion_state_machine_container_[draco_states::kDoubleSupportSwaying]
      ->SetParameters(cfg);

  locomotion_state_machine_container_[draco_states::kLFContactTransitionStart] =
      new ContactTransitionStart(draco_states::kLFContactTransitionStart,
                                 robot_, this);
  locomotion_state_machine_container_[draco_states::kLFContactTransitionStart]
      ->SetParameters(cfg);

  locomotion_state_machine_container_[draco_states::kLFContactTransitionEnd] =
      new ContactTransitionEnd(draco_states::kLFContactTransitionEnd, robot_,
                               this);
  locomotion_state_machine_container_[draco_states::kLFContactTransitionEnd]
      ->SetParameters(cfg);

  locomotion_state_machine_container_[draco_states::kLFSingleSupportSwing] =
      new SingleSupportSwing(draco_states::kLFSingleSupportSwing, robot_, this);
  locomotion_state_machine_container_[draco_states::kLFSingleSupportSwing]
      ->SetParameters(cfg);

  locomotion_state_machine_container_[draco_states::kRFContactTransitionStart] =
      new ContactTransitionStart(draco_states::kRFContactTransitionStart,
                                 robot_, this);
  locomotion_state_machine_container_[draco_states::kRFContactTransitionStart]
      ->SetParameters(cfg);

  locomotion_state_machine_container_[draco_states::kRFContactTransitionEnd] =
      new ContactTransitionEnd(draco_states::kRFContactTransitionEnd, robot_,
                               this);
  locomotion_state_machine_container_[draco_states::kRFContactTransitionEnd]
      ->SetParameters(cfg);

  locomotion_state_machine_container_[draco_states::kRFSingleSupportSwing] =
      new SingleSupportSwing(draco_states::kRFSingleSupportSwing, robot_, this);
  locomotion_state_machine_container_[draco_states::kRFSingleSupportSwing]
      ->SetParameters(cfg);

#if B_USE_TELEOP
  // Manipulation
  manipulation_state_machine_container_[draco_states::kTeleopManipulation] =
      new TeleopManipulation(draco_states::kTeleopManipulation, robot_, this);
  manipulation_state_machine_container_[draco_states::kTeleopManipulation]
      ->SetParameters(cfg);
#endif
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
  delete locomotion_state_machine_container_[draco_states::kInitialize];
  delete locomotion_state_machine_container_
      [draco_states::kDoubleSupportStandUp];
  delete locomotion_state_machine_container_
      [draco_states::kDoubleSupportBalance];
  delete locomotion_state_machine_container_
      [draco_states::kDoubleSupportSwaying];
  delete locomotion_state_machine_container_
      [draco_states::kLFContactTransitionStart];
  delete locomotion_state_machine_container_
      [draco_states::kRFContactTransitionStart];
  delete locomotion_state_machine_container_
      [draco_states::kLFContactTransitionEnd];
  delete locomotion_state_machine_container_
      [draco_states::kRFContactTransitionEnd];
  delete locomotion_state_machine_container_
      [draco_states::kLFSingleSupportSwing];
  delete locomotion_state_machine_container_
      [draco_states::kRFSingleSupportSwing];
#if B_USE_TELEOP
  delete manipulation_state_machine_container_
      [draco_states::kTeleopManipulation];
#endif

#if B_USE_FOXGLOVE
  delete param_subscriber_;
  delete topic_subscriber_;

#endif
}

void DracoControlArchitecture::GetCommand(void *command) {
  if (b_loco_state_first_visit_) {
    locomotion_state_machine_container_[loco_state_]->FirstVisit();
    b_loco_state_first_visit_ = false;
  }
#if B_USE_TELEOP
  if (b_manip_state_first_visit_) {
    manipulation_state_machine_container_[manip_state_]->FirstVisit();
    b_manip_state_first_visit_ = false;
  }
#endif

#if B_USE_FOXGLOVE
  param_subscriber_->UpdateParameters();
  topic_subscriber_->UpdateTopics();
  topic_subscriber_->Connect();
#endif

#if B_USE_TELEOP
  manipulation_state_machine_container_[manip_state_]->OneStep();
#endif
  // desired trajectory update in state machine
  locomotion_state_machine_container_[loco_state_]->OneStep();
  // state independent upper body traj setting
  upper_body_tm_->UseNominalUpperBodyJointPos(sp_->nominal_jpos_);
  // get control command
  controller_->GetCommand(command);

  if (locomotion_state_machine_container_[loco_state_]->EndOfState()) {
    locomotion_state_machine_container_[loco_state_]->LastVisit();
    prev_loco_state_ = loco_state_;
    loco_state_ =
        locomotion_state_machine_container_[loco_state_]->GetNextState();
    b_loco_state_first_visit_ = true;
  }

#if B_USE_TELEOP
  if (manipulation_state_machine_container_[manip_state_]->EndOfState()) {
    manipulation_state_machine_container_[manip_state_]->LastVisit();
    prev_manip_state_ = manip_state_;
    manip_state_ =
        manipulation_state_machine_container_[manip_state_]->GetNextState();
    b_manip_state_first_visit_ = true;
  }
#endif
}

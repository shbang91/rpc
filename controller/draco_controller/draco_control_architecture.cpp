#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_controller.hpp"
#include "controller/draco_controller/draco_crbi/draco_composite_rigid_body_inertia.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_machines/contact_transition_end.hpp"
#include "controller/draco_controller/draco_state_machines/contact_transition_start.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_balance.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_stand_up.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_swaying.hpp"
#include "controller/draco_controller/draco_state_machines/initialize.hpp"
#include "controller/draco_controller/draco_state_machines/locomotion.hpp"
#include "controller/draco_controller/draco_state_machines/single_support_swing.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/qp_params_manager.hpp"
#include "controller/whole_body_controller/managers/reaction_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/upper_body_trajectory_manager.hpp"
#include "convex_mpc/convex_mpc_locomotion.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"
#include "util/util.hpp"

#include "controller/whole_body_controller/task.hpp"
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

  // dcm planner
  dcm_planner_ = new DCMPlanner();

  // convex mpc planner
  YAML::Node cfg_mpc =
      YAML::LoadFile(THIS_COM "config/draco/MPC_LOCOMOTION.yaml");
  double iterations_between_mpc =
      util::ReadParameter<double>(cfg_mpc, "iterations_btw_mpc");
  bool b_save_mpc_solution =
      util::ReadParameter<bool>(cfg_mpc, "b_save_mpc_solution");

  mpc_params_ = new MPCParams();
  Eigen::VectorXd temp_vec;
  double temp_val;
  util::ReadParameter(cfg_mpc["mpc_params"]["mpc_cost"], "Qqq", temp_vec);
  mpc_params_->Qqq_ = temp_vec;
  util::ReadParameter(cfg_mpc["mpc_params"]["mpc_cost"], "Qvv", temp_vec);
  mpc_params_->Qvv_ = temp_vec;
  util::ReadParameter(cfg_mpc["mpc_params"]["mpc_cost"], "Quu", temp_vec);
  mpc_params_->Quu_ = temp_vec;
  util::ReadParameter(cfg_mpc["mpc_params"]["mpc_cost"], "Qqq_terminal",
                      temp_vec);
  mpc_params_->Qqq_terminal_ = temp_vec;
  util::ReadParameter(cfg_mpc["mpc_params"]["mpc_cost"], "Qvv_terminal",
                      temp_vec);
  mpc_params_->Qvv_terminal_ = temp_vec;
  util::ReadParameter(cfg_mpc["mpc_params"]["mpc_cost"], "decay_rate",
                      temp_val);
  mpc_params_->decay_rate_ = temp_val;
  util::ReadParameter(cfg_mpc["mpc_params"], "nominal_inertia", temp_vec);
  mpc_params_->nominal_inertia_ = temp_vec;
  util::ReadParameter(cfg_mpc["mpc_params"]["contact_wrench_cone"], "mu",
                      temp_val);
  mpc_params_->mu_ = temp_val;
  util::ReadParameter(cfg_mpc["mpc_params"]["contact_wrench_cone"], "fz_min",
                      temp_val);
  mpc_params_->fz_min_ = temp_val;
  util::ReadParameter(cfg_mpc["mpc_params"]["contact_wrench_cone"], "fz_max",
                      temp_val);
  mpc_params_->fz_max_ = temp_val;
  util::ReadParameter(cfg_mpc["mpc_params"]["contact_wrench_cone"],
                      "foot_half_length", temp_val);
  mpc_params_->foot_half_length_ = temp_val;
  util::ReadParameter(cfg_mpc["mpc_params"]["contact_wrench_cone"],
                      "foot_half_width", temp_val);
  mpc_params_->foot_half_width_ = temp_val;

  draco_crbi_model_ = new DracoCompositeRigidBodyInertia();

  convex_mpc_locomotion_ = new ConvexMPCLocomotion(
      sp_->servo_dt_, iterations_between_mpc, robot_, b_save_mpc_solution,
      mpc_params_, draco_crbi_model_);

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

  qp_pm_ = new QPParamsManager(tci_container_->qp_params_);

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

  state_machine_container_[draco_states::kLocomotion] =
      new Locomotion(draco_states::kLocomotion, robot_, this);

  this->_InitializeParameters();
}

DracoControlArchitecture::~DracoControlArchitecture() {
  delete tci_container_;
  delete controller_;
  delete dcm_planner_;
  delete convex_mpc_locomotion_;
  delete draco_crbi_model_;

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
  delete qp_pm_;

  // state machines
  delete state_machine_container_[draco_states::kInitialize];
  delete state_machine_container_[draco_states::kDoubleSupportStandUp];
  delete state_machine_container_[draco_states::kDoubleSupportBalance];
  delete state_machine_container_[draco_states::kDoubleSupportSwaying];
  delete state_machine_container_[draco_states::kLFContactTransitionStart];
  delete state_machine_container_[draco_states::kRFContactTransitionStart];
  delete state_machine_container_[draco_states::kLFContactTransitionEnd];
  delete state_machine_container_[draco_states::kRFContactTransitionEnd];
  delete state_machine_container_[draco_states::kLFSingleSupportSwing];
  delete state_machine_container_[draco_states::kRFSingleSupportSwing];

  delete state_machine_container_[draco_states::kLocomotion];
}

void DracoControlArchitecture::GetCommand(void *command) {
  if (b_state_first_visit_) {
    state_machine_container_[state_]->FirstVisit();
    b_state_first_visit_ = false;
  }

  state_machine_container_[state_]->OneStep();
  upper_body_tm_->UseNominalUpperBodyJointPos(
      sp_->nominal_jpos_); // state independent upper body traj setting

  Eigen::VectorXd des_wbo_quat_vec(4);
  des_wbo_quat_vec << 0., 0., 0., 1;
  tci_container_->task_map_["wbo_task"]->UpdateDesired(
      des_wbo_quat_vec, Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3));
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

  state_machine_container_[draco_states::kLFContactTransitionStart]
      ->SetParameters(cfg_);
  state_machine_container_[draco_states::kRFContactTransitionStart]
      ->SetParameters(cfg_);
  state_machine_container_[draco_states::kLFContactTransitionEnd]
      ->SetParameters(cfg_);
  state_machine_container_[draco_states::kRFContactTransitionEnd]
      ->SetParameters(cfg_);

  // dcm planner params initialization
  dcm_tm_->InitializeParameters(cfg_["dcm_walking"]);
}

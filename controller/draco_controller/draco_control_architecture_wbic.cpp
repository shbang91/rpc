#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "configuration.hpp"
#include "controller/draco_controller/draco_control_architecture_wbic.hpp"
#include "controller/draco_controller/draco_controller.hpp"
#include "controller/draco_controller/draco_crbi/draco_composite_rigid_body_inertia.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_machines_wbic/contact_transition_end.hpp"
#include "controller/draco_controller/draco_state_machines_wbic/contact_transition_start.hpp"
#include "controller/draco_controller/draco_state_machines_wbic/double_support_balance.hpp"
#include "controller/draco_controller/draco_state_machines_wbic/double_support_stand_up.hpp"
#include "controller/draco_controller/draco_state_machines_wbic/double_support_swaying.hpp"
#include "controller/draco_controller/draco_state_machines_wbic/initialize.hpp"
#include "controller/draco_controller/draco_state_machines_wbic/single_support_swing.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/whole_body_controller/managers/dcm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/qp_params_manager.hpp"
#include "controller/whole_body_controller/managers/reaction_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/upper_body_trajectory_manager.hpp"
#include "controller/whole_body_controller/task.hpp"
#include "planner/locomotion/dcm_planner/dcm_planner.hpp"

#if B_USE_HPIPM
#include "controller/draco_controller/draco_state_machines_wbic/mpc_locomotion.hpp"
#include "convex_mpc/convex_mpc_locomotion.hpp"
#endif

#if B_USE_FOXGLOVE
#include "UI/foxglove/client/parameter_subscriber.hpp"
#endif

DracoControlArchitecture_WBIC::DracoControlArchitecture_WBIC(
    PinocchioRobotSystem *robot, const YAML::Node &cfg)
    : ControlArchitecture(robot) {
  util::PrettyConstructor(1, "DracoControlArchitecture_WBIC");

  sp_ = DracoStateProvider::GetStateProvider();

  // set starting state
  std::string test_env_name = util::ReadParameter<std::string>(cfg, "env");
  if (test_env_name == "pybullet") {
    prev_loco_state_ = draco_states_wbic::kDoubleSupportStandUp;
    loco_state_ = draco_states_wbic::kDoubleSupportStandUp;
  } else {
    // mujoco & hw
    prev_loco_state_ = draco_states_wbic::kInitialize;
    loco_state_ = draco_states_wbic::kInitialize;
  }

  //=============================================================
  // initialize task, contact, controller, planner
  //=============================================================
  tci_container_ = new DracoTCIContainer(robot_, cfg);
  controller_ = new DracoController(tci_container_, robot_, cfg);

  // dcm planner
  dcm_planner_ = new DCMPlanner();

#if B_USE_HPIPM
  //=============================================================
  // Convex MPC Planner
  //=============================================================
  YAML::Node cfg_mpc;
  if (test_env_name == "mujoco") {
    cfg_mpc = YAML::LoadFile(
        THIS_COM "config/draco/sim/mujoco/wbic/MPC_LOCOMOTION.yaml");
  } else if (test_env_name == "pybullet") {
    cfg_mpc = YAML::LoadFile(
        THIS_COM "config/draco/sim/pybullet/wbic/MPC_LOCOMOTION.yaml");
  } else if (test_env_name == "hw") {
    cfg_mpc =
        YAML::LoadFile(THIS_COM "config/draco/hw/wbic/MPC_LOCOMOTION.yaml");
  } else {
    std::cout
        << "[DracoControlArchitecture_WBIC] Please check the test_env_name"
        << '\n';
    assert(false);
  }

  // mpc gait parameter setting
  mpc_gait_params_ = new GaitParams();
  mpc_gait_params_->x_vel_cmd_ =
      util::ReadParameter<double>(cfg_mpc["gait"], "x_vel_cmd");
  mpc_gait_params_->y_vel_cmd_ =
      util::ReadParameter<double>(cfg_mpc["gait"], "y_vel_cmd");
  mpc_gait_params_->yaw_rate_cmd_ =
      util::ReadParameter<double>(cfg_mpc["gait"], "yaw_rate_cmd");
  mpc_gait_params_->gait_number_ =
      util::ReadParameter<int>(cfg_mpc["gait"], "gait_number");
  mpc_gait_params_->swing_height_ =
      util::ReadParameter<double>(cfg_mpc["swing_foot"], "height");
  mpc_gait_params_->raibert_gain_ =
      util::ReadParameter<double>(cfg_mpc["swing_foot"], "raibert_gain");
  mpc_gait_params_->high_speed_turning_gain_ = util::ReadParameter<double>(
      cfg_mpc["swing_foot"], "high_speed_turning_gain");
  mpc_gait_params_->landing_foot_offset_ = util::ReadParameter<Eigen::Vector3d>(
      cfg_mpc["swing_foot"], "landing_foot_offset");

  // mpc parameter setting
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
#endif

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

  qp_pm_ = new QPParamsManager(tci_container_->qp_params_);

  //=============================================================
  // initialize state machines
  //=============================================================
  locomotion_state_machine_container_[draco_states_wbic::kInitialize] =
      new Initialize_WBIC(draco_states_wbic::kInitialize, robot_, this);
  locomotion_state_machine_container_[draco_states_wbic::kInitialize]
      ->SetParameters(cfg);

  locomotion_state_machine_container_
      [draco_states_wbic::kDoubleSupportStandUp] =
          new DoubleSupportStandUp_WBIC(
              draco_states_wbic::kDoubleSupportStandUp, robot_, this);
  locomotion_state_machine_container_[draco_states_wbic::kDoubleSupportStandUp]
      ->SetParameters(cfg);

  locomotion_state_machine_container_
      [draco_states_wbic::kDoubleSupportBalance] =
          new DoubleSupportBalance_WBIC(
              draco_states_wbic::kDoubleSupportBalance, robot_, this);

  locomotion_state_machine_container_
      [draco_states_wbic::kDoubleSupportSwaying] =
          new DoubleSupportSwaying_WBIC(
              draco_states_wbic::kDoubleSupportSwaying, robot_, this);
  locomotion_state_machine_container_[draco_states_wbic::kDoubleSupportSwaying]
      ->SetParameters(cfg);

  locomotion_state_machine_container_
      [draco_states_wbic::kLFContactTransitionStart] =
          new ContactTransitionStart_WBIC(
              draco_states_wbic::kLFContactTransitionStart, robot_, this);
  locomotion_state_machine_container_
      [draco_states_wbic::kLFContactTransitionStart]
          ->SetParameters(cfg);

  locomotion_state_machine_container_
      [draco_states_wbic::kLFContactTransitionEnd] =
          new ContactTransitionEnd_WBIC(
              draco_states_wbic::kLFContactTransitionEnd, robot_, this);
  locomotion_state_machine_container_
      [draco_states_wbic::kLFContactTransitionEnd]
          ->SetParameters(cfg);

  locomotion_state_machine_container_
      [draco_states_wbic::kLFSingleSupportSwing] = new SingleSupportSwing_WBIC(
          draco_states_wbic::kLFSingleSupportSwing, robot_, this);
  locomotion_state_machine_container_[draco_states_wbic::kLFSingleSupportSwing]
      ->SetParameters(cfg);

  locomotion_state_machine_container_
      [draco_states_wbic::kRFContactTransitionStart] =
          new ContactTransitionStart_WBIC(
              draco_states_wbic::kRFContactTransitionStart, robot_, this);
  locomotion_state_machine_container_
      [draco_states_wbic::kRFContactTransitionStart]
          ->SetParameters(cfg);

  locomotion_state_machine_container_
      [draco_states_wbic::kRFContactTransitionEnd] =
          new ContactTransitionEnd_WBIC(
              draco_states_wbic::kRFContactTransitionEnd, robot_, this);
  locomotion_state_machine_container_
      [draco_states_wbic::kRFContactTransitionEnd]
          ->SetParameters(cfg);

  locomotion_state_machine_container_
      [draco_states_wbic::kRFSingleSupportSwing] = new SingleSupportSwing_WBIC(
          draco_states_wbic::kRFSingleSupportSwing, robot_, this);
  locomotion_state_machine_container_[draco_states_wbic::kRFSingleSupportSwing]
      ->SetParameters(cfg);

#if B_USE_HPIPM
  locomotion_state_machine_container_[draco_states_wbic::kMPCLocomotion] =
      new MPCLocomotion_WBIC(draco_states_wbic::kMPCLocomotion, robot_, this);
  locomotion_state_machine_container_[draco_states_wbic::kMPCLocomotion]
      ->SetParameters(cfg);
#endif
}

DracoControlArchitecture_WBIC::~DracoControlArchitecture_WBIC() {
  delete tci_container_;
  delete controller_;
  delete dcm_planner_;

#if B_USE_HPIPM
  // mpc
  delete mpc_gait_params_;
  delete mpc_params_;
  delete draco_crbi_model_;
  delete convex_mpc_locomotion_;
#endif

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
  delete locomotion_state_machine_container_[draco_states_wbic::kInitialize];
  delete locomotion_state_machine_container_
      [draco_states_wbic::kDoubleSupportStandUp];
  delete locomotion_state_machine_container_
      [draco_states_wbic::kDoubleSupportBalance];
  delete locomotion_state_machine_container_
      [draco_states_wbic::kDoubleSupportSwaying];
  delete locomotion_state_machine_container_
      [draco_states_wbic::kLFContactTransitionStart];
  delete locomotion_state_machine_container_
      [draco_states_wbic::kRFContactTransitionStart];
  delete locomotion_state_machine_container_
      [draco_states_wbic::kLFContactTransitionEnd];
  delete locomotion_state_machine_container_
      [draco_states_wbic::kRFContactTransitionEnd];
  delete locomotion_state_machine_container_
      [draco_states_wbic::kLFSingleSupportSwing];
  delete locomotion_state_machine_container_
      [draco_states_wbic::kRFSingleSupportSwing];

#if B_USE_HPIPM
  delete locomotion_state_machine_container_[draco_states_wbic::kMPCLocomotion];
#endif
}

void DracoControlArchitecture_WBIC::GetCommand(void *command) {
  if (b_loco_state_first_visit_) {
    locomotion_state_machine_container_[loco_state_]->FirstVisit();
    b_loco_state_first_visit_ = false;
  }

  locomotion_state_machine_container_[loco_state_]->OneStep();
  upper_body_tm_->UseNominalUpperBodyJointPos(
      sp_->nominal_jpos_); // state independent upper body traj setting

  // TODO: make it general
  // tci_container_->task_map_["wbo_task"]->UpdateDesired(
  // sp_->wbo_des_, Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3));
  controller_->GetCommand(command); // get control command

  if (locomotion_state_machine_container_[loco_state_]->EndOfState()) {
    locomotion_state_machine_container_[loco_state_]->LastVisit();
    prev_loco_state_ = loco_state_;
    loco_state_ =
        locomotion_state_machine_container_[loco_state_]->GetNextState();
    b_loco_state_first_visit_ = true;
  }
}

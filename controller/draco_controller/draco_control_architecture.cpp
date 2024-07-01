#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_controller.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_balance.hpp"
#include "controller/draco_controller/draco_state_machines/double_support_stand_up.hpp"
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

#include "controller/draco_controller/draco_state_machines/alip_locomotion.hpp"
#include "controller/whole_body_controller/managers/alipmpc_trajectory_manager.hpp"

DracoControlArchitecture::DracoControlArchitecture(PinocchioRobotSystem *robot)
    : ControlArchitecture(robot) {
  // util::PrettyConstructor(1, "DracoControlArchitecture");

  sp_ = DracoStateProvider::GetStateProvider();

  try {
    cfg_ = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }

  bool b_sim = util::ReadParameter<bool>(cfg_, "b_sim");
  util::ReadParameter(cfg_["alip_mpc_walking"], "verbose", verbose);

  // set starting state
  /*
  prev_state_ =
      b_sim ? draco_states::kDoubleSupportStandUp : draco_states::kInitialize;
  state_ =
      b_sim ? draco_states::kDoubleSupportStandUp : draco_states::kInitialize;
  */
  state_ = draco_states::kDoubleSupportStandUp;
  prev_state_ = draco_states::kDoubleSupportStandUp;

  std::string prefix = b_sim ? "sim" : "exp";

  //=============================================================
  // initialize task, contact, controller, planner
  //=============================================================
  tci_container_ = new DracoTCIContainer(robot_);
  controller_ = new DracoController(tci_container_, robot_);

  dcm_planner_ = new DCMPlanner();

  std::string step_horizon;
  std::string intervals;
  bool new_solver;
  util::ReadParameter(cfg_["alip_mpc_walking"], "step_horizon", step_horizon);
  util::ReadParameter(cfg_["alip_mpc_walking"], "intervals", intervals);
  util::ReadParameter(cfg_["alip_mpc_walking"], "new_solver", new_solver);
  alip_mpc_ = new NewStep_mpc(step_horizon, intervals, new_solver);
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
  alip_tm_ = new AlipMpcTrajectoryManager( // initialises the planner also
      alip_mpc_, tci_container_->task_map_["com_xy_task"],
      tci_container_->task_map_["com_z_task"],
      tci_container_->task_map_["torso_ori_task"],
      tci_container_->task_map_["lf_pos_task"],
      tci_container_->task_map_["lf_ori_task"],
      tci_container_->task_map_["rf_pos_task"],
      tci_container_->task_map_["rf_ori_task"],
      tci_container_->force_task_map_["lf_force_task"],
      tci_container_->force_task_map_["rf_force_task"], robot_);

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
  state_machine_container_[draco_states::AlipLocomotion] =
      new AlipLocomotion(draco_states::AlipLocomotion, robot_, this);
  alipIter = 0;
  this->_InitializeParameters();
  sp_->outsideCommand(cfg_["alip_mpc_walking"]);
}

DracoControlArchitecture::~DracoControlArchitecture() {
  delete tci_container_;
  delete controller_;
  delete dcm_planner_;
  delete alip_mpc_;

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
  delete alip_tm_;

  // state machines
  delete state_machine_container_[draco_states::kInitialize];
  delete state_machine_container_[draco_states::kDoubleSupportStandUp];
  delete state_machine_container_[draco_states::AlipLocomotion];
}

void DracoControlArchitecture::GetCommand(void *command) {
  if (state_ == draco_states::AlipLocomotion) {
    //  util::PrettyConstructor(1,"ctrlarch GetCommand ") ;

    if (alipIter == 0) {
      state_machine_container_[draco_states::AlipLocomotion]->FirstVisit();
      first_ever = false;
    }
    if (first_ever && alipIter == 2) {
      alip_tm_->saveDoubleStanceFoot();
    }

    if (alipIter >= 0)
      state_machine_container_[draco_states::AlipLocomotion]->OneStep();
    if (alipIter < 0) {
      alip_tm_->saveDoubleStanceFoot();
      alip_tm_->UpdateDoubleStance();
      tci_container_->contact_map_["rf_contact"]->SetMaxFz(310);
      tci_container_->contact_map_["lf_contact"]->SetMaxFz(310);
      tci_container_->task_map_["lf_pos_task"]->SetWeight(
          8500 * Eigen::VectorXd::Ones(3));
      tci_container_->task_map_["rf_pos_task"]->SetWeight(
          8500 * Eigen::VectorXd::Ones(3));
      tci_container_->task_map_["rf_ori_task"]->SetWeight(
          6500 * Eigen::VectorXd::Ones(3));
      tci_container_->task_map_["lf_ori_task"]->SetWeight(
          6500 * Eigen::VectorXd::Ones(3));
    }
    upper_body_tm_->UseNominalUpperBodyJointPos(sp_->nominal_jpos_);
    controller_->GetCommand(command);
    alipIter++;
    if (sp_->mpc_freq_ != 0 && alipIter == sp_->mpc_freq_) {
      alipIter = 0;
      sp_->rl_trigger_ = true;
    }
    if (verbose == true) {
      alip_tm_->saveRobotCommand(sp_->current_time_);
      alip_tm_->saveCurrentCOMstate(sp_->current_time_);
      alip_tm_->saveMpcCOMstate(sp_->current_time_);
      alip_tm_->saveSwingState(sp_->current_time_);
      alip_tm_->saveCOMstateWorld(sp_->current_time_);
      alip_tm_->saveCOMstateMPCcoor(sp_->current_time_);
    }

    if (state_machine_container_[draco_states::AlipLocomotion]->SwitchLeg()) {
      // alipIter = 0;
      alipIter = -3;
      // exit(0);
      // cfg_ = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
      // sp_->outsideCommand(cfg_["alip_mpc_walking"]);
      sp_->rl_trigger_ = true;
      first_ever = true;
    }

    //  std::cout << "ctroarch end Get Command" << std::endl << std::endl;

  } else {
    if (b_state_first_visit_) {
      state_machine_container_[state_]->FirstVisit();
      b_state_first_visit_ = false;
    }

    state_machine_container_[state_]->OneStep();
    upper_body_tm_->UseNominalUpperBodyJointPos(
        sp_->nominal_jpos_); // state independent upper body traj setting
    controller_->GetCommand(command); // get control command

    if (state_machine_container_[state_]->EndOfState()) {
      state_machine_container_[state_]->LastVisit();
      prev_state_ = state_;
      state_ = state_machine_container_[state_]->GetNextState();
      b_state_first_visit_ = true;
      if (state_ == draco_states::AlipLocomotion) {
        sp_->rl_trigger_ = true;
        alip_tm_->initializeOri();
        alip_tm_->setNewOri(sp_->des_com_yaw_);
        sp_->des_end_torso_iso_ = alip_tm_->Get_des_end_torso_iso();
        alip_tm_->saveDoubleStanceFoot();
      }
    }
  }
  save_freq_++;
  if (verbose && save_freq_ == SAVE_FREQ_) {
    tci_container_->saveTxts(sp_->current_time_, sp_->stance_leg_, sp_->state_);
    save_freq_ = 0;
  }
  // this->Reset();
}

void DracoControlArchitecture::_InitializeParameters() {
  // state machine initialization
  state_machine_container_[draco_states::kInitialize]->SetParameters(
      cfg_["state_machine"]["initialize"]);
  state_machine_container_[draco_states::kDoubleSupportStandUp]->SetParameters(
      cfg_["state_machine"]["stand_up"]);
  state_machine_container_[draco_states::AlipLocomotion]->SetParameters(
      cfg_["alip_mpc_walking"]);

  // dcm planner params initialization
  alip_tm_->SetParameters(cfg_["alip_mpc_walking"]);
  alip_tm_->SetTaskWeights(cfg_["alip_mpc_walking"]["task"]);
  alip_mpc_->SetParameters(cfg_["alip_mpc_walking"]);

  util::ReadParameter(cfg_["alip_mpc_walking"], "rf_z_MAX", rf_MAX_);
  util::ReadParameter(cfg_["alip_mpc_walking"], "save_freq", SAVE_FREQ_);
  // draco_control_arch
  // util::ReadParameter(cfg_["alip_mpc_walking"], "mpc_freq", mpc_freq_);
}

void DracoControlArchitecture::Reset() {
  state_ = draco_states::kDoubleSupportStandUp;
  prev_state_ = draco_states::kDoubleSupportStandUp;
  alipIter = 0;
  b_state_first_visit_ = true;
  state_machine_container_[draco_states::AlipLocomotion]->Reset();
  // controller_->Reset();

  // alip_tm_->initializeOri();
}

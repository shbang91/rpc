#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/optimo_controller/optimo_control_architecture.hpp"
#include "controller/optimo_controller/optimo_controller.hpp"
#include "controller/optimo_controller/optimo_definition.hpp"

#include "controller/optimo_controller/optimo_state_machines/initialize.hpp"
#include "controller/optimo_controller/optimo_state_machines/stand_up.hpp"
#include "controller/optimo_controller/optimo_state_machines/ee_traj.hpp"
#include "controller/optimo_controller/optimo_state_machines/task_transition.hpp"

#include "controller/optimo_controller/optimo_state_provider.hpp"
#include "controller/optimo_controller/optimo_tci_container.hpp"

#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/arm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/reaction_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "controller/whole_body_controller/managers/upper_body_trajectory_manager.hpp"
#include "util/util.hpp"

OptimoControlArchitecture::OptimoControlArchitecture(
    PinocchioRobotSystem *robot)
    : ControlArchitecture(robot) {
  util::PrettyConstructor(1, "OptimoControlArchitecture");

  sp_ = OptimoStateProvider::GetStateProvider();

  try {
    cfg_ = YAML::LoadFile(THIS_COM "config/optimo/ihwbc_gains.yaml");
  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }

  bool b_sim = util::ReadParameter<bool>(cfg_, "b_sim");

  prev_state_ = optimo_states::kInitialize;
  state_ = optimo_states::kInitialize;

  std::string prefix = b_sim ? "sim" : "exp";

  //=============================================================
  // initialize task, contact, controller, planner
  //=============================================================
  tci_container_ = new OptimoTCIContainer(robot_);
  controller_ = new OptimoController(tci_container_, robot_);

  //=============================================================
  // trajectory Managers
  //=============================================================
  //  initialize kinematics manager

  // upper_body_tm_ = new UpperBodyTrajetoryManager(
  // tci_container_->task_map_["upper_body_task"], robot_);

  ee_SE3_tm_ = new ArmTrajectoryManager(
      tci_container_->task_map_["ee_pos_task"],
      tci_container_->task_map_["ee_ori_task"], robot_);

  
//   f1_SE3_tm_ = new EndEffectorTrajectoryManager(
//       tci_container_->task_map_["f1_ee_pos_task"],
//       tci_container_->task_map_["f1_ee_ori_task"], robot_);

//   f2_SE3_tm_ = new EndEffectorTrajectoryManager(
//       tci_container_->task_map_["f2_ee_pos_task"],
//       tci_container_->task_map_["f2_ee_ori_task"], robot_);

//   f3_SE3_tm_ = new EndEffectorTrajectoryManager(
//       tci_container_->task_map_["f3_ee_pos_task"],
//       tci_container_->task_map_["f3_ee_ori_task"], robot_);

  Eigen::VectorXd weight, weight_min;


  // Joint Position Task
  try{
    util::ReadParameter(cfg_["wbc"]["task"]["jpos_task"], prefix + "_weight",
                        weight);
    util::ReadParameter(cfg_["wbc"]["task"]["jpos_task"],
                        prefix + "_weight_min", weight_min);

    jpos_hm_ = new TaskHierarchyManager(
    tci_container_->task_map_["jpos_task"], weight, weight_min);
    
  } catch (const std::runtime_error &ex) {
    std::cerr << "Error reading parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }


  try {
    util::ReadParameter(cfg_["wbc"]["task"]["ee_pos_task"], prefix + "_weight",
                        weight);
    util::ReadParameter(cfg_["wbc"]["task"]["ee_pos_task"],
                        prefix + "_weight_min", weight_min);

    ee_pos_hm_ = new TaskHierarchyManager(
    tci_container_->task_map_["ee_pos_task"], weight, weight_min);

    util::ReadParameter(cfg_["wbc"]["task"]["f1_pos_task"], prefix + "_weight",
                        weight);
    util::ReadParameter(cfg_["wbc"]["task"]["f1_pos_task"],
                        prefix + "_weight_min", weight_min);

    util::ReadParameter(cfg_["wbc"]["task"]["f2_pos_task"], prefix + "_weight",
                        weight);
    util::ReadParameter(cfg_["wbc"]["task"]["f2_pos_task"],
                        prefix + "_weight_min", weight_min);

    util::ReadParameter(cfg_["wbc"]["task"]["f3_pos_task"], prefix + "_weight",
                        weight);
    util::ReadParameter(cfg_["wbc"]["task"]["f3_pos_task"],
                        prefix + "_weight_min", weight_min);

  } catch (const std::runtime_error &ex) {
    std::cerr << "Error reading parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  // f1_pos_hm_ = new TaskHierarchyManager(
  // tci_container_->task_map_["f1_pos_task"], weight, weight_min);

  // f2_pos_hm_ = new TaskHierarchyManager(
  // tci_container_->task_map_["f2_pos_task"], weight, weight_min);

  // f3_pos_hm_ = new TaskHierarchyManager(
  // tci_container_->task_map_["f3_pos_task"], weight, weight_min);

  try {
    util::ReadParameter(cfg_["wbc"]["task"]["ee_ori_task"], prefix + "_weight",
                        weight);
    util::ReadParameter(cfg_["wbc"]["task"]["ee_ori_task"],
                        prefix + "_weight_min", weight_min);

    ee_ori_hm_ = new TaskHierarchyManager(
    tci_container_->task_map_["ee_ori_task"], weight, weight_min);

    util::ReadParameter(cfg_["wbc"]["task"]["f1_ori_task"], prefix + "_weight",
                        weight);
    util::ReadParameter(cfg_["wbc"]["task"]["f1_ori_task"],
                        prefix + "_weight_min", weight_min);

    util::ReadParameter(cfg_["wbc"]["task"]["f2_ori_task"], prefix + "_weight",
                        weight);
    util::ReadParameter(cfg_["wbc"]["task"]["f2_ori_task"],
                        prefix + "_weight_min", weight_min);

    util::ReadParameter(cfg_["wbc"]["task"]["f3_ori_task"], prefix + "_weight",
                        weight);
    util::ReadParameter(cfg_["wbc"]["task"]["f3_ori_task"],
                        prefix + "_weight_min", weight_min);

  } catch (const std::runtime_error &ex) {
    std::cerr << "Error reading parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  // f1_ori_hm_ = new TaskHierarchyManager(
  // tci_container_->task_map_["f1_ori_task"], weight, weight_min);

  // f2_ori_hm_ = new TaskHierarchyManager(
  // tci_container_->task_map_["f2_ori_task"], weight, weight_min);

  // f3_ori_hm_ = new TaskHierarchyManager(
  // tci_container_->task_map_["f3_ori_task"], weight, weight_min);

  double max_rf_z;
  util::ReadParameter(cfg_["wbc"]["contact"], prefix + "_max_rf_z", max_rf_z);

  // ee_force_tm_ = new ForceTrajectoryManager(
  // tci_container_->force_task_map_["ee_force_task"], robot_);

  // f1_force_tm_ = new ForceTrajectoryManager(
  // tci_container_->force_task_map_["f1_force_task"], robot_);
  // f2_force_tm_ = new ForceTrajectoryManager(
  //     tci_container_->force_task_map_["f2_force_task"], robot_);
  // f3_force_tm_ = new ForceTrajectoryManager(
  //     tci_container_->force_task_map_["f3_force_task"], robot_);

  //=============================================================
  // initialize state machines
  //=============================================================
  state_machine_container_[optimo_states::kInitialize] =
      new Initialize(optimo_states::kInitialize, robot_, this);

  state_machine_container_[optimo_states::kTaskTransition] =
      new TaskTransition(optimo_states::kTaskTransition, robot_, this);

  state_machine_container_[optimo_states::kEETraj] =
      new EETraj(optimo_states::kEETraj, robot_, this);


  this->_InitializeParameters();
}

void OptimoControlArchitecture::GetCommand(void *command) {
  if (b_state_first_visit_) {
    state_machine_container_[state_]->FirstVisit();
    b_state_first_visit_ = false;
  }

  state_machine_container_[state_]->OneStep();
  controller_->GetCommand(command); // get control command

  if (state_machine_container_[state_]->EndOfState()) {
    state_machine_container_[state_]->LastVisit();
    prev_state_ = state_;
    state_ = state_machine_container_[state_]->GetNextState();
    b_state_first_visit_ = true;
  }

}

void OptimoControlArchitecture::_InitializeParameters() {
  // state machine initialization
  state_machine_container_[optimo_states::kInitialize]->SetParameters(
      cfg_["state_machine"]["initialize"]);
  state_machine_container_[optimo_states::kTaskTransition]->SetParameters(
      cfg_["state_machine"]["task_transition"]);
  state_machine_container_[optimo_states::kEETraj]->SetParameters(
      cfg_["state_machine"]["ee_traj"]);
}

OptimoControlArchitecture::~OptimoControlArchitecture() {
  delete tci_container_;
  delete controller_;

  // tm
  // delete upper_body_tm_;
  delete ee_SE3_tm_;
//   delete f1_SE3_tm_;
//   delete f2_SE3_tm_;
//   delete f3_SE3_tm_;

  // delete ee_force_tm_;
  // delete f1_force_tm_;
  // delete f2_force_tm_;
  // delete f3_force_tm_;

  // hm
  delete jpos_hm_;
  delete ee_pos_hm_;
  delete ee_ori_hm_;
//   delete f1_pos_hm_;
//   delete f1_ori_hm_;
//   delete f2_pos_hm_;
//   delete f2_ori_hm_;
//   delete f3_pos_hm_;
//   delete f3_ori_hm_;

  // state machines
  delete state_machine_container_[optimo_states::kInitialize];
  delete state_machine_container_[optimo_states::kTaskTransition];
  delete state_machine_container_[optimo_states::kEETraj];

}

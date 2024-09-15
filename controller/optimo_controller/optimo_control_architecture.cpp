#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/optimo_controller/optimo_control_architecture.hpp"
#include "controller/optimo_controller/optimo_controller.hpp"
#include "controller/optimo_controller/optimo_definition.hpp"

#include "controller/optimo_controller/optimo_state_machines/ee_traj.hpp"
#include "controller/optimo_controller/optimo_state_machines/initialize.hpp"
#include "controller/optimo_controller/optimo_state_machines/task_transition.hpp"

#include "controller/optimo_controller/optimo_state_provider.hpp"
#include "controller/optimo_controller/optimo_tci_container.hpp"

#include "controller/whole_body_controller/managers/arm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "util/util.hpp"

OptimoControlArchitecture::OptimoControlArchitecture(
    PinocchioRobotSystem *robot, const YAML::Node &cfg)
    : ControlArchitecture(robot) {
  util::PrettyConstructor(1, "OptimoControlArchitecture");

  sp_ = OptimoStateProvider::GetStateProvider();

  // set starting state
  prev_manip_state_ = optimo_states::kInitialize;
  manip_state_ = optimo_states::kInitialize;

  //=============================================================
  // initialize task, contact, controller, planner
  //=============================================================
  tci_container_ = new OptimoTCIContainer(robot_, cfg);
  controller_ = new OptimoController(tci_container_, robot_, cfg);

  //=============================================================
  // trajectory Managers
  //=============================================================
  //  initialize kinematics manager
  ee_SE3_tm_ = new ArmTrajectoryManager(
      tci_container_->task_map_["ee_pos_task"],
      tci_container_->task_map_["ee_ori_task"], robot_);

  Eigen::VectorXd weight, weight_min;
  try {
    // Joint Position Task
    util::ReadParameter(cfg["wbc"]["task"]["jpos_task"], "weight", weight);
    util::ReadParameter(cfg["wbc"]["task"]["jpos_task"], "weight_min",
                        weight_min);
    jpos_hm_ = new TaskHierarchyManager(tci_container_->task_map_["jpos_task"],
                                        weight, weight_min);

    // EE Position Task
    util::ReadParameter(cfg["wbc"]["task"]["ee_pos_task"], "weight", weight);
    util::ReadParameter(cfg["wbc"]["task"]["ee_pos_task"], "weight_min",
                        weight_min);
    ee_pos_hm_ = new TaskHierarchyManager(
        tci_container_->task_map_["ee_pos_task"], weight, weight_min);

    // EE Orientation Task
    util::ReadParameter(cfg["wbc"]["task"]["ee_ori_task"], "weight", weight);
    util::ReadParameter(cfg["wbc"]["task"]["ee_ori_task"], "weight_min",
                        weight_min);
    ee_ori_hm_ = new TaskHierarchyManager(
        tci_container_->task_map_["ee_ori_task"], weight, weight_min);

  } catch (const std::runtime_error &ex) {
    std::cerr << "Error reading parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }

  //=============================================================
  // initialize state machines
  //=============================================================
  manipulation_state_machine_container_[optimo_states::kInitialize] =
      new Initialize(optimo_states::kInitialize, robot_, this);
  manipulation_state_machine_container_[optimo_states::kInitialize]
      ->SetParameters(cfg);

  manipulation_state_machine_container_[optimo_states::kTaskTransition] =
      new TaskTransition(optimo_states::kTaskTransition, robot_, this);
  manipulation_state_machine_container_[optimo_states::kTaskTransition]
      ->SetParameters(cfg);

  manipulation_state_machine_container_[optimo_states::kEETraj] =
      new EETraj(optimo_states::kEETraj, robot_, this);
  manipulation_state_machine_container_[optimo_states::kEETraj]->SetParameters(
      cfg);
}

OptimoControlArchitecture::~OptimoControlArchitecture() {
  delete tci_container_;
  delete controller_;

  // tm
  delete ee_SE3_tm_;

  // hm
  delete jpos_hm_;
  delete ee_pos_hm_;
  delete ee_ori_hm_;

  // state machines
  delete manipulation_state_machine_container_[optimo_states::kInitialize];
  delete manipulation_state_machine_container_[optimo_states::kTaskTransition];
  delete manipulation_state_machine_container_[optimo_states::kEETraj];
}

void OptimoControlArchitecture::GetCommand(void *command) {
  if (b_manip_state_first_visit_) {
    manipulation_state_machine_container_[manip_state_]->FirstVisit();
    b_manip_state_first_visit_ = false;
  }

  manipulation_state_machine_container_[manip_state_]->OneStep();
  controller_->GetCommand(command); // get control command

  if (manipulation_state_machine_container_[manip_state_]->EndOfState()) {
    manipulation_state_machine_container_[manip_state_]->LastVisit();
    prev_manip_state_ = manip_state_;
    manip_state_ =
        manipulation_state_machine_container_[manip_state_]->GetNextState();
    b_manip_state_first_visit_ = true;
  }
}

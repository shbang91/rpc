#include "controller/med7_controller/med7_state_machines/task_transition.hpp"
#include "controller/med7_controller/med7_control_architecture.hpp"
#include "controller/med7_controller/med7_definition.hpp"
#include "controller/med7_controller/med7_state_provider.hpp"
#include "controller/med7_controller/med7_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/managers/arm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"

#include "util/interpolation.hpp"

TaskTransition::TaskTransition(const StateId state_id,
                               PinocchioRobotSystem *robot,
                               Med7ControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch), b_stay_here_(false),
      wait_time_(0.) {
  util::PrettyConstructor(2, "TaskTransition");

  sp_ = Med7StateProvider::GetStateProvider();

  // Construct Initial and Target Isometric Matrix
  init_iso_ = Eigen::Isometry3d::Identity();
  target_iso_ = Eigen::Isometry3d::Identity();
}

TaskTransition::~TaskTransition() {}

void TaskTransition::FirstVisit() {
  state_machine_start_time_ = sp_->current_time_;
  std::cout << "med7_states::kTaskTransition" << std::endl;

  // Get current state of the robot
  init_iso_ = robot_->GetLinkIsometry(med7_link::link_inst_ee);
  // Set Target se3 for task transition
  target_iso_ = sp_->des_ee_iso_;

  // Initialize se3 trajectory
  ctrl_arch_->ee_SE3_tm_->InitializeTrajectory(init_iso_, target_iso_,
                                               end_time_);

  // Initialize Task Hierarchy Manager
  ctrl_arch_->ee_pos_hm_->InitializeRampToMax(end_time_);
  ctrl_arch_->ee_ori_hm_->InitializeRampToMax(end_time_);
  ctrl_arch_->jpos_hm_->InitializeRampToMin(end_time_);

  // Hard Transition
  // ctrl_arch_->ee_pos_hm_->UpdateInstantToMax();
  // ctrl_arch_->ee_ori_hm_->UpdateInstantToMax();
  // ctrl_arch_->jpos_hm_->UpdateInstantToMin();
}

void TaskTransition::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // Update Task Hierarchy Manager
  ctrl_arch_->ee_pos_hm_->UpdateRampToMax(state_machine_time_);
  ctrl_arch_->ee_ori_hm_->UpdateRampToMax(state_machine_time_);
  ctrl_arch_->jpos_hm_->UpdateRampToMin(state_machine_time_);

  // Update Task Trajectory
  ctrl_arch_->ee_SE3_tm_->UpdateDesired(sp_->current_time_);
}

void TaskTransition::LastVisit() {}

bool TaskTransition::EndOfState() {
  if (b_stay_here_) {
    return false;
  } else {
    return (state_machine_time_ >= end_time_ + wait_time_) ? true : false;
  }
}

StateId TaskTransition::GetNextState() { return med7_states::kEETraj; }

void TaskTransition::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "duration", end_time_);
    util::ReadParameter(node, "b_stay_here", b_stay_here_);
    util::ReadParameter(node, "wait_time", wait_time_);
  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

#include "controller/med7_controller/med7_state_machines/initialize.hpp"
#include "controller/med7_controller/med7_control_architecture.hpp"
#include "controller/med7_controller/med7_definition.hpp"
#include "controller/med7_controller/med7_state_provider.hpp"
#include "controller/med7_controller/med7_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/whole_body_controller/managers/arm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "controller/whole_body_controller/basic_task.hpp"



#include "util/interpolation.hpp"

Initialize::Initialize(const StateId state_id, PinocchioRobotSystem *robot,
                       Med7ControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch), b_stay_here_(false),
      wait_time_(0.), min_jerk_curves_(nullptr) {
  util::PrettyConstructor(2, "Initialize");

  sp_ = Med7StateProvider::GetStateProvider();
  target_joint_pos_ = Eigen::VectorXd::Zero(robot_->NumActiveDof());
  init_joint_pos_ = Eigen::VectorXd::Zero(robot_->NumActiveDof());

}

Initialize::~Initialize() {
  if (min_jerk_curves_ != nullptr) {
    delete min_jerk_curves_;
  }
}

void Initialize::FirstVisit() {
  std::cout << "med7_states::kInitialize" << std::endl;

  // Get Current state of the robot
  state_machine_start_time_ = sp_->current_time_;
  init_joint_pos_ = robot_->GetJointPos();


  // Construct Desired Joint Trajectory
  min_jerk_curves_ = new MinJerkCurveVec(
      init_joint_pos_, 
      Eigen::VectorXd::Zero(init_joint_pos_.size()),
      Eigen::VectorXd::Zero(init_joint_pos_.size()), 
      target_joint_pos_,
      Eigen::VectorXd::Zero(target_joint_pos_.size()),
      Eigen::VectorXd::Zero(target_joint_pos_.size()), end_time_); // min jerk curve initialization

  // Set EE Task Gain to Min
  ctrl_arch_->ee_pos_hm_->UpdateInstantToMin();
  ctrl_arch_->ee_ori_hm_->UpdateInstantToMin();

}

void Initialize::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  Eigen::VectorXd des_joint_pos =
      Eigen::VectorXd::Zero(target_joint_pos_.size());
  Eigen::VectorXd des_joint_vel =
      Eigen::VectorXd::Zero(target_joint_pos_.size());
  Eigen::VectorXd des_joint_acc =
      Eigen::VectorXd::Zero(target_joint_pos_.size());

  if (min_jerk_curves_ == nullptr)
    throw std::runtime_error(
        "Initialize MinJerkCurve in Initialize StateMachine");

  for (unsigned int i(0); i < target_joint_pos_.size(); ++i) {
    des_joint_pos = min_jerk_curves_->Evaluate(state_machine_time_);
    des_joint_vel =
        min_jerk_curves_->EvaluateFirstDerivative(state_machine_time_);
    des_joint_acc =
        min_jerk_curves_->EvaluateSecondDerivative(state_machine_time_);
        
  }
  
  // Update Joint Position Task
  ctrl_arch_->tci_container_->task_map_["jpos_task"]->UpdateDesired(
      des_joint_pos, des_joint_vel, des_joint_acc);

}

void Initialize::LastVisit() {
 sp_->des_ee_iso_ = robot_->GetLinkIsometry(med7_link::link_inst_ee);

}

bool Initialize::EndOfState() {
  if (b_stay_here_) {
    return false;
  } else {
    return (state_machine_time_ >= end_time_ + wait_time_) ? true : false;
  }
}

StateId Initialize::GetNextState() { return med7_states::kTaskTransition; }

void Initialize::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "init_duration", end_time_);
    util::ReadParameter(node, "target_joint_pos", target_joint_pos_);
    sp_->nominal_jpos_ = target_joint_pos_; // set nominal jpos
    util::ReadParameter(node, "b_stay_here", b_stay_here_);
    util::ReadParameter(node, "wait_time", wait_time_);
    util::ReadParameter(node, "task_transit_time", task_transit_time_);

  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

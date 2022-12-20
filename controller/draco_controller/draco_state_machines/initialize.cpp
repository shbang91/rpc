#include "controller/draco_controller/draco_state_machines/initialize.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"

Initialize::Initialize(const StateId state_id, PinocchioRobotSystem *robot,
                       DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch), duration_(0.) {
  util::PrettyConstructor(2, "Initialize");

  sp_ = DracoStateProvider::GetStateProvider();
  target_joint_pos_ = Eigen::VectorXd::Zero(robot_->NumActiveDof());
  init_joint_pos_ = Eigen::VectorXd::Zero(robot_->NumActiveDof());
}

void Initialize::FirstVisit() {
  std::cout << "draco_states::kInitialize" << std::endl;
  state_machine_start_time_ = sp_->current_time_;
  init_joint_pos_ = robot_->GetJointPos();
  min_jerk_curves_ = MinJerkCurveVec(
      init_joint_pos_, Eigen::VectorXd::Zero(init_joint_pos_.size()),
      Eigen::VectorXd::Zero(init_joint_pos_.size()), target_joint_pos_,
      Eigen::VectorXd::Zero(target_joint_pos_.size()),
      Eigen::VectorXd::Zero(target_joint_pos_.size()),
      duration_); // min jerk curve initialization
}

void Initialize::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  Eigen::VectorXd des_joint_pos =
      Eigen::VectorXd::Zero(target_joint_pos_.size());
  Eigen::VectorXd des_joint_vel =
      Eigen::VectorXd::Zero(target_joint_pos_.size());
  Eigen::VectorXd des_joint_acc =
      Eigen::VectorXd::Zero(target_joint_pos_.size());
  for (unsigned int i(0); i < target_joint_pos_.size(); ++i) {
    des_joint_pos = min_jerk_curves_.Evaluate(state_machine_time_);
    des_joint_vel =
        min_jerk_curves_.EvaluateFirstDerivative(state_machine_time_);
    des_joint_acc =
        min_jerk_curves_.EvaluateSecondDerivative(state_machine_time_);
  }
  ctrl_arch_->tci_container_->jpos_task_->UpdateDesired(
      des_joint_pos, des_joint_vel, des_joint_acc);
}

void Initialize::LastVisit() {}

bool Initialize::EndOfState() {
  if (b_stay_here_) {
    return false;
  } else {
    return (state_machine_time_ >= duration_) ? true : false;
  }
}

StateId Initialize::GetNextState() {
  return draco_states::kDoubleSupportStandUp;
}

void Initialize::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "init_duration", duration_);
    util::ReadParameter(node, "target_joint_pos", target_joint_pos_);
    sp_->nominal_jpos_ = target_joint_pos_; // set nominal jpos
    util::ReadParameter(node, "b_only_joint_pos_control", b_stay_here_);

  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

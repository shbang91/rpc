#include "controller/draco_controller/draco_state_machines/initialize.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "util/interpolation.hpp"

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
}

void Initialize::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  Eigen::VectorXd des_joint_pos(
      Eigen::VectorXd::Zero(target_joint_pos_.size()));
  Eigen::VectorXd des_joint_vel(
      Eigen::VectorXd::Zero(target_joint_pos_.size()));
  Eigen::VectorXd des_joint_acc(
      Eigen::VectorXd::Zero(target_joint_pos_.size()));
  for (unsigned int i(0); i < target_joint_pos_.size(); ++i) {
    des_joint_pos[i] = util::SmoothPos(init_joint_pos_[i], target_joint_pos_[i],
                                       duration_, state_machine_time_);
    des_joint_vel[i] = util::SmoothVel(init_joint_pos_[i], target_joint_pos_[i],
                                       duration_, state_machine_time_);
    des_joint_pos[i] = util::SmoothAcc(init_joint_pos_[i], target_joint_pos_[i],
                                       duration_, state_machine_time_);
  }
  ctrl_arch_->tci_container_->jpos_task_->UpdateDesired(
      des_joint_pos, des_joint_vel, des_joint_acc);
}

void Initialize::LastVisit() {}

bool Initialize::EndOfState() {
  return (state_machine_time_ >= duration_) ? true : false;
}

StateId Initialize::GetNextState() {
  return draco_states::kDoubleSupportStandUp;
}

void Initialize::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "init_duration", duration_);
    util::ReadParameter(node, "target_joint_pos", target_joint_pos_);

    // set nominal jpos
    sp_->nominal_jpos_ = target_joint_pos_;

  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

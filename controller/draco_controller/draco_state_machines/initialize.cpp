#include "controller/draco_controller/draco_state_machines/initialize.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "util/interpolation.hpp"

Initialize::Initialize(const StateId state_id, PinocchioRobotSystem *robot,
                       DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch), b_stay_here_(false),
      wait_time_(0.), min_jerk_curves_(nullptr) {
  util::PrettyConstructor(2, "Initialize");

  sp_ = DracoStateProvider::GetStateProvider();
  target_joint_pos_ = Eigen::VectorXd::Zero(robot_->NumActiveDof());
  init_joint_pos_ = Eigen::VectorXd::Zero(robot_->NumActiveDof());
}

Initialize::~Initialize() {
  if (min_jerk_curves_ != nullptr)
    delete min_jerk_curves_;
}

void Initialize::FirstVisit() {
  std::cout << "draco_states::kInitialize" << std::endl;
  state_machine_start_time_ = sp_->current_time_;
  init_joint_pos_ = robot_->GetJointPos();
  min_jerk_curves_ = new MinJerkCurveVec(
      init_joint_pos_, Eigen::VectorXd::Zero(init_joint_pos_.size()),
      Eigen::VectorXd::Zero(init_joint_pos_.size()), target_joint_pos_,
      Eigen::VectorXd::Zero(target_joint_pos_.size()),
      Eigen::VectorXd::Zero(target_joint_pos_.size()),
      end_time_); // min jerk curve initialization
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
  ctrl_arch_->tci_container_->task_map_["joint_task"]->UpdateDesired(
      des_joint_pos, des_joint_vel, des_joint_acc);
}

void Initialize::LastVisit() {}

bool Initialize::EndOfState() {
  if (b_stay_here_) {
    return false;
  } else {
    return (state_machine_time_ >= end_time_ + wait_time_) ? true : false;
  }
}

StateId Initialize::GetNextState() {
  return draco_states::kDoubleSupportStandUp;
}

void Initialize::SetParameters(const YAML::Node &cfg) {
  try {
    util::ReadParameter(cfg["state_machine"]["initialize"], "init_duration",
                        end_time_);
    util::ReadParameter(cfg["state_machine"]["initialize"], "target_joint_pos",
                        target_joint_pos_);
    sp_->nominal_jpos_ = target_joint_pos_; // set nominal jpos
    util::ReadParameter(cfg["state_machine"]["initialize"],
                        "b_only_joint_pos_control", b_stay_here_);
    util::ReadParameter(cfg["state_machine"]["initialize"], "wait_time",
                        wait_time_);

  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

#include "pnc/fixed_draco_pnc/fixed_draco_state_machines/initialize.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_control_architecture.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_tci_container.hpp"
#include "pnc/robot_system/robot_system.hpp"
#include "util/interpolation.hpp"

Initialize::Initialize(StateId _state_id,
                       FixedDracoControlArchitecture *_ctrl_arch,
                       RobotSystem *_robot)
    : StateMachine(_state_id, _robot) {
  std::cout << "initialize" << std::endl;
  state_id_ = _state_id;
  robot_ = _robot;
  ctrl_arch_ = _ctrl_arch;

  sp_ = FixedDracoStateProvider::GetStateProvider();

  ini_jpos_.resize(robot_->n_a_);
  des_jpos_.resize(robot_->n_a_);
  duration_ = 0.;
}

Initialize::~Initialize() {}

void Initialize::FirstVisit() {
  std::cout << "fixedDracoState::kInitialize" << std::endl;
  state_machine_start_time_ = sp_->current_time;
  ini_jpos_ = robot_->joint_positions_;
}

void Initialize::OneStep() {
  state_machine_time_ = sp_->current_time - state_machine_start_time_;
  Eigen::VectorXd des_jpos, des_jvel, des_jacc;
  des_jpos.resize(robot_->n_a_);
  des_jvel.resize(robot_->n_a_);
  des_jacc.resize(robot_->n_a_);

  for (int i = 0; i < robot_->n_a_; i++) {
    des_jpos[i] = util::SmoothPos(ini_jpos_[i], des_jpos_[i], duration_,
                                  state_machine_time_);
    des_jvel[i] = util::SmoothVel(ini_jpos_[i], des_jpos_[i], duration_,
                                  state_machine_time_);
    des_jacc[i] = util::SmoothAcc(ini_jpos_[i], des_jpos_[i], duration_,
                                  state_machine_time_);
  }
  ctrl_arch_->tci_container_->jpos_task_->UpdateDesiredTask(des_jpos, des_jvel,
                                                            des_jacc);
}

bool Initialize::EndOfState() {
  // if (state_machine_time_ > duration_) {
  // return true;
  //}
  return false;
}

void Initialize::LastVisit() {}

StateId Initialize::GetNextState() { return FixedDracoState::kRightFootMove; }

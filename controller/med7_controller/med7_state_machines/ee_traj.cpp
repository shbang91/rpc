#include "controller/med7_controller/med7_state_machines/ee_traj.hpp"
#include "controller/med7_controller/med7_control_architecture.hpp"
#include "controller/med7_controller/med7_definition.hpp"
#include "controller/med7_controller/med7_state_provider.hpp"
#include "controller/med7_controller/med7_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/managers/arm_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"

#include "util/interpolation.hpp"

EETraj::EETraj(const StateId state_id, PinocchioRobotSystem *robot,
                       Med7ControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch), b_stay_here_(false),
      wait_time_(0.) {
  util::PrettyConstructor(2, "EETraj");

    sp_ = Med7StateProvider::GetStateProvider();

    // Initialize Target Position and Orientation
    target_pos_ = Eigen::Vector3d::Zero();
    target_ori_ = Eigen::Vector4d::Zero();

    // Construct Initial and Target Isometric Matrix
    init_iso_ = robot_->GetLinkIsometry(med7_link::link_inst_ee);
    target_iso_ = Eigen::Isometry3d::Identity();
  
}

EETraj::~EETraj() {

}

void EETraj::FirstVisit(){
    state_machine_start_time_ = sp_->current_time_;
    std::cout << "med7_states::kEETraj" << std::endl;
    init_iso_ = robot_->GetLinkIsometry(med7_link::link_inst_ee);

    // Construct Target Isometric Matrix from target position and quaternion
    Eigen::Quaterniond quat(target_ori_(0), target_ori_(1), target_ori_(2), target_ori_(3));
    target_iso_.linear() = quat.toRotationMatrix();
    target_iso_.translation() = target_pos_;

    // Initialize EE traj
    ctrl_arch_->ee_SE3_tm_->InitializeTrajectory(init_iso_, target_iso_, end_time_);

    // print target orientation
    Eigen::Vector3d rpy = util::RPYFromSO3(init_iso_.linear());
    std::cout << "Initial Orientation: " << rpy.transpose() << std::endl;

    

}

void EETraj::OneStep() {
    state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

    // update EE trajectory
    ctrl_arch_->ee_SE3_tm_->UpdateDesired(state_machine_time_);
}

void EETraj::LastVisit() {
}

bool EETraj::EndOfState() {
  if (b_stay_here_) {
    return false;
  } else {
    return (state_machine_time_ >= end_time_ + wait_time_) ? true : false;
  }
}

StateId EETraj::GetNextState() { return med7_states::kStandUp; }

void EETraj::SetParameters(const YAML::Node &node) {
    try {
        util::ReadParameter(node, "duration", end_time_);
        util::ReadParameter(node, "target_pos", target_pos_);
        util::ReadParameter(node, "target_ori", target_ori_);
        util::ReadParameter(node, "b_stay_here", b_stay_here_);
        util::ReadParameter(node, "wait_time", wait_time_); 
    } catch (const std::runtime_error &e) {
        std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
                  << __FILE__ << "]" << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

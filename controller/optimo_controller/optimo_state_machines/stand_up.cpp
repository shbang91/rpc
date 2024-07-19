#include "controller/optimo_controller/optimo_state_machines/stand_up.hpp"
#include "controller/optimo_controller/optimo_control_architecture.hpp"
#include "controller/optimo_controller/optimo_state_provider.hpp"
#include "controller/optimo_controller/optimo_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "util/interpolation.hpp"

StandUp::StandUp(const StateId state_id, PinocchioRobotSystem *robot, 
                        OptimoControlArchitecture *ctrl_arch) 
        : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch), b_stay_here_(false), 
        wait_time_(0.), min_jerk_curves_(nullptr) {
    util::PrettyConstructor(2, "StandUp");

    sp_ = OptimoStateProvider::GetStateProvider();
    target_joint_pos_ = Eigen::VectorXd::Zero(robot_->NumActiveDof());
    init_joint_pos_ = Eigen::VectorXd::Zero(robot_->NumActiveDof());
}

StandUp::~StandUp() {
     if (min_jerk_curves_ != nullptr) {
        delete min_jerk_curves_;
    }
}
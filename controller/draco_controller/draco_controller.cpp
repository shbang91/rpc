#include "controller/draco_controller/draco_controller.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_interface.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"

DracoController::DracoController(DracoTCIContainer *tci_container,
                                 PinocchioRobotSystem *robot)
    : tci_container_(tci_container), robot_(robot) {
  util::PrettyConstructor(2, "DracoController");
  sp_ = DracoStateProvider::GetStateProvider();
}

DracoController::~DracoController() {}

void DracoController::GetCommand(void *command) {
  if (sp_->state_ == draco_states::kInitialize) {
    // joint position control command
    (static_cast<DracoCommand *>(command))->joint_pos_cmd_ =
        tci_container_->jpos_task_->GetTaskDesiredPos();
    (static_cast<DracoCommand *>(command))->joint_vel_cmd_ =
        tci_container_->jpos_task_->GetTaskDesiredVel();
    (static_cast<DracoCommand *>(command))->joint_vel_cmd_ =
        Eigen::VectorXd::Zero(draco::n_adof);
  } else {
    // whole body controller (feedforward torque computation)
  }
}

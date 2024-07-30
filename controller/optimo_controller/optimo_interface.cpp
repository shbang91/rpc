#include "configuration.hpp"

#include "util/util.hpp"

#include "controller/robot_system/pinocchio_robot_system.hpp"


#include "controller/optimo_controller/optimo_interface.hpp"
#include "controller/optimo_controller/optimo_control_architecture.hpp"
#include "controller/optimo_controller/optimo_state_estimator.hpp"
#include "controller/optimo_controller/optimo_task_gain_handler.hpp"

OptimoInterface::OptimoInterface() : Interface() {
  std::string border = "=";
  for (unsigned int i = 0; i < 79; ++i)
    border += "=";
  util::ColorPrint(color::kBoldRed, border);
  util::PrettyConstructor(0, "OptimoInterface");

  robot_ = new PinocchioRobotSystem(THIS_COM "robot_model/optimo/optimo.urdf",
                                    THIS_COM "robot_model/optimo", true, false);
  se_ = new OptimoStateEstimator(robot_);

  ctrl_arch_ = new OptimoControlArchitecture(robot_);

  // interrupt_handler_ = new InterruptHandler(static_cast<OptimoControlArchitecture *>(ctrl_arch_));
  task_gain_handler_ = new OptimoTaskGainHandler(static_cast<OptimoControlArchitecture *>(ctrl_arch_));
}

OptimoInterface::~OptimoInterface() { 
  delete robot_;
  delete se_;
  delete ctrl_arch_;
  // delete interrupt_handler_;
  delete task_gain_handler_;
}

void OptimoInterface::GetCommand(void *sensor_data, void *command_data) {
  // TODO:timing variables

  OptimoSensorData *optimo_sensor_data =
      static_cast<OptimoSensorData *>(sensor_data);

  OptimoCommand *optimo_command = static_cast<OptimoCommand *>(command_data);

  // update sensor data to pinocchio model using state estimator class
  se_->Update(optimo_sensor_data);

  // TODO:get control command through control architecture class

  // process interrupt & task gains

  // TODO: interrupt handler
  // if (interrupt_handler_->IsSignalReceived()) {
  //     interrupt_handler_->Process();
  // }

  // get control command
  ctrl_arch_->GetCommand(optimo_command);

  count_++;
}

void OptimoInterface::_SafeCommand(OptimoSensorData *data, OptimoCommand *command) {
  command->joint_pos_cmd_ = data->joint_pos_;
  command->joint_vel_cmd_.setZero();
  command->joint_trq_cmd_.setZero();
}

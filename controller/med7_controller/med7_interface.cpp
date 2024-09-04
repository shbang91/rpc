#include "configuration.hpp"

#include "util/util.hpp"

#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/med7_controller/med7_state_estimator.hpp"

#include "controller/med7_controller/med7_control_architecture.hpp"
#include "controller/med7_controller/med7_interface.hpp"
#include "controller/med7_controller/med7_interrupt_handler.hpp"
#include "controller/med7_controller/med7_state_provider.hpp"
#include "controller/med7_controller/med7_task_gain_handler.hpp"

#include "controller/med7_controller/med7_definition.hpp"

Med7Interface::Med7Interface() : Interface() {
  std::string border = "=";
  for (unsigned int i = 0; i < 79; ++i)
    border += "=";
  util::ColorPrint(color::kBoldRed, border);
  util::PrettyConstructor(0, "Med7Interface");

  sp_ = Med7StateProvider::GetStateProvider();
  try {
    YAML::Node cfg = YAML::LoadFile(
        THIS_COM "config/med7/ihwbc_gains.yaml"); // get yaml node

    sp_->servo_dt_ =
        util::ReadParameter<double>(cfg, "servo_dt"); // set control frequency
  } catch (const std::runtime_error &ex) {
    std::cerr << "Error Reading Parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }

  robot_ = new PinocchioRobotSystem(THIS_COM "robot_model/med7/urdf/med7.urdf",
                                    THIS_COM "robot_model/med7", true, false);
  se_ = new Med7StateEstimator(robot_);

  ctrl_arch_ = new Med7ControlArchitecture(robot_);

  interrupt_handler_ = new Med7InterruptHandler(
      static_cast<Med7ControlArchitecture *>(ctrl_arch_));
  task_gain_handler_ = new Med7TaskGainHandler(
      static_cast<Med7ControlArchitecture *>(ctrl_arch_));
}

Med7Interface::~Med7Interface() {
  delete robot_;
  delete se_;
  delete ctrl_arch_;
  delete interrupt_handler_;
  delete task_gain_handler_;
}

void Med7Interface::GetCommand(void *sensor_data, void *command_data) {
  sp_->count_ = count_;
  sp_->current_time_ = static_cast<double>(count_) * sp_->servo_dt_;
  sp_->state_ = ctrl_arch_->state();
  sp_->prev_state_ = ctrl_arch_->prev_state();

  Med7SensorData *med7_sensor_data = static_cast<Med7SensorData *>(sensor_data);

  Med7Command *med7_command = static_cast<Med7Command *>(command_data);

  // update sensor data to pinocchio model using state estimator class
  se_->Update(med7_sensor_data);

  // process interrupt & task gains
  if (interrupt_handler_->IsSignalReceived()) {
    interrupt_handler_->Process();
  }
  if (task_gain_handler_->IsSignalReceived())
    task_gain_handler_->Process();

  // get control command through control architecture class
  ctrl_arch_->GetCommand(med7_command);

  count_++;
}

void Med7Interface::_SafeCommand(Med7SensorData *data, Med7Command *command) {
  command->joint_pos_cmd_ = data->joint_pos_;
  command->joint_vel_cmd_.setZero();
  command->joint_trq_cmd_.setZero();
}

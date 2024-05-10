#include "configuration.hpp"

#include "util/util.hpp"

#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/optimo_controller/optimo_interface.hpp"

OptimoInterface::OptimoInterface() : Interface() {
  std::string border = "=";
  for (unsigned int i = 0; i < 79; ++i)
    border += "=";
  util::ColorPrint(color::kBoldRed, border);
  util::PrettyConstructor(0, "OptimoInterface");

  robot_ = new PinocchioRobotSystem(THIS_COM "robot_model/optimo/optimo.urdf",
                                    THIS_COM "robot_model/optimo", true, false);
}

OptimoInterface::~OptimoInterface() { delete robot_; }

void OptimoInterface::GetCommand(void *sensor_data, void *command_data) {
  // TODO:timing variables

  OptimoSensorData *optimo_sensor_data =
      static_cast<OptimoSensorData *>(sensor_data);
  OptimoCommand *optimo_command = static_cast<OptimoCommand *>(command_data);

  // TODO:update sensor data to pinocchio model using state estimator class

  // TODO:get control command through control architecture class
}

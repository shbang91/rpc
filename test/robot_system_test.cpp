#include <iostream>

#include "pnc/robot_system/pinocchio_robot_system.hpp"

#include "configuration.hpp"
#include "pnc/robot_system/dart_robot_system.hpp"

int main() {
  RobotSystem *dart_robot;
  dart_robot = new DartRobotSystem(
      THIS_COM "robot_model/atlas/atlas_rel_path.urdf", false, false);

  RobotSystem *pin_robot;
  pin_robot =
      new PinocchioRobotSystem(THIS_COM "robot_model/atlas/atlas.urdf",
                               THIS_COM "robot_model/atlas", false, false);

  std::cout << "============DART=============" << std::endl;
  std::cout << "joint pos: " << std::endl;
  std::cout << dart_robot->joint_positions_ << std::endl;
  std::cout << "lfoot_jac: " << std::endl;
  std::cout << dart_robot->GetLinkJacobian("r_sole") << std::endl;

  std::cout << "============PINOCCHIO=============" << std::endl;
  std::cout << "joint pos: " << std::endl;
  std::cout << pin_robot->joint_positions_ << std::endl;
  std::cout << "lfoot_jac: " << std::endl;
  std::cout << pin_robot->GetLinkJacobian("r_sole") << std::endl;

  delete dart_robot;
  delete pin_robot;

  return 0;
}

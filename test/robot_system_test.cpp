#include <iostream>

#include "configuration.hpp"
#include "pnc/robot_system/dart_robot_system.hpp"
#include "pnc/robot_system/pinocchio_robot_system.hpp"

int main() {
  RobotSystem *dart_robot;
  dart_robot = new DartRobotSystem(
      THIS_COM "robot_model/draco/draco_rel_path.urdf", true, false);

  RobotSystem *pin_robot;
  pin_robot =
      new PinocchiRobotSystem(THIS_COM "robot_model/atals/atlas.urdf",
                              THIS_COM "robot_model/atals", false, true);

  std::cout << "============DART=============" << std::endl;
  std::cout << "joint pos: " << robot->joint_positions_ << std::endl;
  std::cout << "lfoot_jac: " << std::endl;
  std::cout << robot->GetLinkJacobian("l_foot_contact") << std::endl;

  std::cout << "============PINOCCHIO=============" << std::endl;
  std::cout << "joint pos: " << robot->joint_positions_ << std::endl;
  std::cout << "lfoot_jac: " << std::endl;
  std::cout << robot->GetLinkJacobian("l_foot_contact") << std::endl;

  delete robot;

  return 0;
}

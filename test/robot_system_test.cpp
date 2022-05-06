#include <iostream>

#include "configuration.hpp"
#include "pnc/robot_system/dart_robot_system.hpp"

int main() {
  RobotSystem *robot;
  robot = new DartRobotSystem(THIS_COM "robot_model/draco/draco_rel_path.urdf",
                              true, false);
  std::cout << "joint pos: " << robot->joint_positions_ << std::endl;
  std::cout << "lfoot_jac: " << std::endl;
  std::cout << robot->GetLinkJacobian("l_foot_contact") << std::endl;

  return 0;
}

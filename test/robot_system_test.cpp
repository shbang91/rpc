#include <iostream>
#include <map>
#include <string>

#include "pnc/robot_system/pinocchio_robot_system.hpp"

#include "configuration.hpp"
#include "pnc/robot_system/dart_robot_system.hpp"

int main() {
  RobotSystem *dart_robot;
  dart_robot = new DartRobotSystem(
      THIS_COM "robot_model/atlas/atlas_rel_path.urdf", false, false);

  // RobotSystem *pin_robot;
  // pin_robot =
  // new PinocchioRobotSystem(THIS_COM "robot_model/atlas/atlas.urdf",
  // THIS_COM "robot_model/atlas", false, false);

  Eigen::Vector3d com_pos(0, 0, 0);
  Eigen::Vector3d com_lv(0.1, 0, 0);
  Eigen::Vector3d com_av(0.1, 0, 0);
  Eigen::Quaternion<double> com_quat(1, 0, 0, 0);
  Eigen::Vector3d bjoint_pos(0, 0, 0);
  Eigen::Vector3d bjoint_lv(0.1, 0, 0);
  Eigen::Vector3d bjoint_av(0.1, 0, 0);
  Eigen::Quaternion<double> bjoint_quat(1, 0, 0, 0);

  std::map<std::string, double> joint_pos = {
      {"back_bkx", 0},    {"back_bky", 0},    {"back_bkz", 0},
      {"l_arm_elx", 0},   {"l_arm_ely", 0},   {"l_arm_shx", 0},
      {"l_arm_shz", 0},   {"l_arm_wrx", 0},   {"l_arm_wry", 0},
      {"l_arm_wry2", 0},  {"l_leg_akx", 0},   {"l_leg_aky", 0},
      {"l_leg_hpx", 0},   {"l_leg_hpy", 0},   {"l_leg_hpz", 0},
      {"l_leg_kny", 0},   {"neck_ry", 0},     {"r_arm_elx", 0},
      {"r_arm_ely", 0},   {"r_arm_shx", 0},   {"r_arm_shz", 0},
      {"r_arm_wrx", 0},   {"r_arm_wry", 0},   {"r_arm_wry2", 0},
      {"r_leg_akx", 0.1}, {"r_leg_aky", 0.2}, {"r_leg_hpx", 0},
      {"r_leg_hpy", 0},   {"r_leg_hpz", 0},   {"r_leg_kny", 0}};
  std::map<std::string, double> joint_vel = {
      {"back_bkx", 0},    {"back_bky", 0},    {"back_bkz", 0},
      {"l_arm_elx", 0},   {"l_arm_ely", 0},   {"l_arm_shx", 0},
      {"l_arm_shz", 0},   {"l_arm_wrx", 0},   {"l_arm_wry", 0},
      {"l_arm_wry2", 0},  {"l_leg_akx", 0},   {"l_leg_aky", 0},
      {"l_leg_hpx", 0},   {"l_leg_hpy", 0},   {"l_leg_hpz", 0},
      {"l_leg_kny", 0},   {"neck_ry", 0},     {"r_arm_elx", 0},
      {"r_arm_ely", 0},   {"r_arm_shx", 0},   {"r_arm_shz", 0},
      {"r_arm_wrx", 0},   {"r_arm_wry", 0},   {"r_arm_wry2", 0},
      {"r_leg_akx", 0.1}, {"r_leg_aky", 0.2}, {"r_leg_hpx", 0},
      {"r_leg_hpy", 0},   {"r_leg_hpz", 0},   {"r_leg_kny", 0}};

  dart_robot->UpdateRobotModel(com_pos, com_quat.normalized(), com_lv, com_av,
                               bjoint_pos, bjoint_quat.normalized(), bjoint_lv,
                               bjoint_av, joint_pos, joint_vel, true);

  // pin_robot->UpdateRobotModel(com_pos, com_quat.normalized(), com_lv, com_av,
  // bjoint_pos, bjoint_quat.normalized(), bjoint_lv,
  // bjoint_av, joint_pos, joint_vel, true);

  std::cout << "============DART=============" << std::endl;
  std::cout << "joint pos: " << std::endl;
  std::cout << dart_robot->joint_positions_ << std::endl;
  std::cout << "lfoot_jac: " << std::endl;
  std::cout << dart_robot->GetLinkJacobian("r_sole") << std::endl;

  // std::cout << "============PINOCCHIO=============" << std::endl;
  // std::cout << "joint pos: " << std::endl;
  // std::cout << pin_robot->joint_positions_ << std::endl;
  // std::cout << "lfoot_jac: " << std::endl;
  // std::cout << pin_robot->GetLinkJacobian("r_sole") << std::endl;

  delete dart_robot;
  // delete pin_robot;

  return 0;
}

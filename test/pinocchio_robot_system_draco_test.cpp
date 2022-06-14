#include <chrono>
#include <iostream>
#include <map>
#include <string>

#include "configuration.hpp"
#include "pnc/robot_system/pinocchio_robot_system.hpp"

#include "pnc/draco_pnc/draco_definition.hpp"

int main() {

  PinocchioRobotSystem robot(THIS_COM "robot_model/draco/draco_modified.urdf",
                             THIS_COM "robot_model/draco", false, false, 2);

  Eigen::Vector3d com_pos(0, 0, 0);
  Eigen::Vector3d com_lv(0.1, 0, 0);
  Eigen::Vector3d com_av(0.1, 0, 0);
  Eigen::Quaternion<double> com_quat(1, 0, 0, 0);
  Eigen::Vector3d bjoint_pos(0, 0, 0);
  Eigen::Vector3d bjoint_lv(0.1, 0, 0);
  Eigen::Vector3d bjoint_av(0.1, 0, 0);
  Eigen::Quaternion<double> bjoint_quat(1, 0, 0, 0);

  std::map<std::string, double> joint_pos = {
      {"l_hip_ie", 0},      {"l_hip_aa", 0},      {"l_hip_fe", 0},
      {"l_knee_fe_jp", 0},  {"l_knee_fe_jd", 0},  {"l_ankle_fe", 0},
      {"l_ankle_ie", 0},    {"l_shoulder_fe", 0}, {"l_shoulder_aa", 0},
      {"l_shoulder_ie", 0}, {"l_elbow_fe", 0},    {"l_wrist_ps", 0},
      {"l_wrist_pitch", 0}, {"neck_pitch", 0},    {"r_hip_ie", 0},
      {"r_hip_aa", 0},      {"r_hip_fe", 0},      {"r_knee_fe_jp", 0},
      {"r_knee_fe_jd", 0},  {"r_ankle_fe", 0},    {"r_ankle_ie", 0},
      {"r_shoulder_fe", 0}, {"r_shoulder_aa", 0}, {"r_shoulder_ie", 0},
      {"r_elbow_fe", 0},    {"r_wrist_ps", 0},    {"r_wrist_pitch", 0}};
  std::map<std::string, double> joint_vel = {
      {"l_hip_ie", 0},      {"l_hip_aa", 0},      {"l_hip_fe", 0},
      {"l_knee_fe_jp", 0},  {"l_knee_fe_jd", 0},  {"l_ankle_fe", 0},
      {"l_ankle_ie", 0},    {"l_shoulder_fe", 0}, {"l_shoulder_aa", 0},
      {"l_shoulder_ie", 0}, {"l_elbow_fe", 0},    {"l_wrist_ps", 0},
      {"l_wrist_pitch", 0}, {"neck_pitch", 0},    {"r_hip_ie", 0},
      {"r_hip_aa", 0},      {"r_hip_fe", 0},      {"r_knee_fe_jp", 0},
      {"r_knee_fe_jd", 0},  {"r_ankle_fe", 0},    {"r_ankle_ie", 0},
      {"r_shoulder_fe", 0}, {"r_shoulder_aa", 0}, {"r_shoulder_ie", 0},
      {"r_elbow_fe", 0},    {"r_wrist_ps", 0},    {"r_wrist_pitch", 0}};

  robot.UpdateRobotModel(com_pos, com_quat.normalized(), com_lv, com_av,
                         bjoint_pos, bjoint_quat.normalized(), bjoint_lv,
                         bjoint_av, joint_pos, joint_vel, true);

  std::cout << "============PINOCCHIO robot updated=============" << std::endl;

  // std::cout << "q" << std::endl;
  // std::cout << robot.GetQ() << std::endl;

  // std::cout << "qdot" << std::endl;
  // std::cout << robot.GetQdot() << std::endl;

  // std::cout << "Ig" << std::endl;
  // std::cout << robot.Ig_ << std::endl;

  // std::cout << "Hg" << std::endl;
  // std::cout << robot.Hg_ << std::endl;

  // std::cout << "Ag" << std::endl;
  // std::cout << robot.Ag_ << std::endl;

  // std::cout << "mass matrix" << std::endl;
  // std::cout << robot.GetMassMatrix() << std::endl;

  // std::cout << "gravity" << std::endl;
  // std::cout << robot.GetGravity() << std::endl;

  // std::cout << "coriolis" << std::endl;
  // std::cout << robot.GetCoriolis() << std::endl;

  // std::cout << "rfoot pos" << std::endl;
  // Eigen::Isometry3d iso = robot.GetLinkIso("r_foot_contact");
  // std::cout << iso.translation() << std::endl;
  // std::cout << iso.linear() << std::endl;

  // std::cout << "rfoot vel" << std::endl;
  // Eigen::Matrix<double, 6, 1> vel = robot.GetLinkVel("r_foot_contact");
  // std::cout << vel.transpose() << std::endl;

  // std::cout << "rfoot jac" << std::endl;
  // std::cout << robot.GetLinkJacobian("r_foot_contact") << std::endl;

  // std::cout << "rfoot link acc drift" << std::endl;
  // std::cout << robot.GetLinkJacobianDotQdot("r_foot_contact").transpose()
  //<< std::endl;

  // std::cout << "com" << std::endl;
  // std::cout << robot.GetRobotComPos() << std::endl;

  // std::cout << "com vel" << std::endl;
  // std::cout << robot.GetRobotComLinVel() << std::endl;

  // std::cout << "com jac" << std::endl;
  // std::cout << robot.GetComLinJacobian() << std::endl;

  // std::cout << "com jdot_qdot" << std::endl;
  // std::cout << (robot.GetComLinJacobianDot() * robot.GetQdot()).transpose()
  //<< std::endl;

  return 0;
}

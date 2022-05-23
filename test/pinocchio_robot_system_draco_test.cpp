#include <chrono>
#include <iostream>
#include <map>
#include <string>

#include "configuration.hpp"
#include "pnc/robot_system/pinocchio_robot_system.hpp"

#include "pnc/draco_pnc/draco_definition.hpp"

int main() {

  // PinocchioRobotSystem robot(THIS_COM
  // "robot_model/draco/draco_modified.urdf", THIS_COM "robot_model/draco",
  // false, false, 2);
  // RobotSystem *robot_ = &robot;
  auto tic = std::chrono::high_resolution_clock::now();
  RobotSystem *robot_ =
      new PinocchioRobotSystem(THIS_COM "robot_model/draco/draco_modified.urdf",
                               THIS_COM "robot_model/draco", false, false, 2);
  auto toc = std::chrono::high_resolution_clock::now();
  std::cout << std::chrono::duration<double, std::milli>(toc - tic).count()
            << "ms" << std::endl;
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

  // robot.UpdateRobotModel(com_pos, com_quat.normalized(), com_lv, com_av,
  // bjoint_pos, bjoint_quat.normalized(), bjoint_lv,
  // bjoint_av, joint_pos, joint_vel, true);
  auto tic1 = std::chrono::high_resolution_clock::now();
  robot_->UpdateRobotModel(com_pos, com_quat.normalized(), com_lv, com_av,
                           bjoint_pos, bjoint_quat.normalized(), bjoint_lv,
                           bjoint_av, joint_pos, joint_vel, true);
  auto toc1 = std::chrono::high_resolution_clock::now();
  std::cout << std::chrono::duration<double, std::milli>(toc1 - tic1).count()
            << "ms" << std::endl;

  std::cout << "============PINOCCHIO robot updated=============" << std::endl;

  std::cout << "r_sole pos with string: " << std::endl;
  // auto tic = std::chrono::high_resolution_clock::now();
  Eigen::Isometry3d iso = robot_->GetLinkIso("r_foot_contact");
  // auto toc = std::chrono::high_resolution_clock::now();

  std::cout << iso.linear() << std::endl;
  std::cout << iso.translation() << std::endl;
  // std::cout << "time passed: "
  //<< std::chrono::duration<double, std::milli>(toc - tic).count()
  //<< "ms" << std::endl;

  // std::cout << "r_sole pos with int: " << std::endl;
  // auto tic1 = std::chrono::high_resolution_clock::now();
  // Eigen::Isometry3d iso2 =
  // robot.GetLinkIso(draco_link::r_foot_contact_frame); auto toc1 =
  // std::chrono::high_resolution_clock::now(); std::cout << iso2.linear() <<
  // std::endl; std::cout << iso2.translation() << std::endl; std::cout << "time
  // passed: "
  //<< std::chrono::duration<double, std::milli>(toc1 - tic1).count()
  //<< "ms" << std::endl;

  // Eigen::Isometry3d iso2 =
  // robot_->GetLinkIso(draco_link::r_foot_contact_frame); std::cout <<
  // iso2.linear() << std::endl; std::cout << iso2.translation() << std::endl;

  delete robot_;

  return 0;
}

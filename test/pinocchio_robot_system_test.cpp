#include <chrono>
#include <iostream>
#include <map>
#include <string>

#include "configuration.hpp"
#include "pnc/robot_system/pinocchio_robot_system.hpp"

int main() {
  // RobotSystem *pin_robot =
  // new PinocchioRobotSystem(THIS_COM "robot_model/atlas/atlas.urdf",
  // THIS_COM "robot_model/atlas", false, false);
  PinocchioRobotSystem pin_robot(THIS_COM
                                 "robot_model/draco/draco_modified.urdf",
                                 THIS_COM "robot_model/draco", false, false, 2);

  Eigen::Vector3d com_pos(0, 0, 0);
  Eigen::Vector3d com_lv(0.1, 0, 0);
  Eigen::Vector3d com_av(0.1, 0, 0);
  Eigen::Quaternion<double> com_quat(1, 0, 0, 0);
  Eigen::Vector3d bjoint_pos(0, 0, 0);
  Eigen::Vector3d bjoint_lv(0.1, 0, 0);
  Eigen::Vector3d bjoint_av(0.1, 0, 0);
  Eigen::Quaternion<double> bjoint_quat(1, 0, 0, 0);

  // std::map<std::string, double> joint_pos = {
  //{"back_bkx", 0},    {"back_bky", 0},    {"back_bkz", 0},
  //{"l_arm_elx", 0},   {"l_arm_ely", 0},   {"l_arm_shx", 0},
  //{"l_arm_shz", 0},   {"l_arm_wrx", 0},   {"l_arm_wry", 0},
  //{"l_arm_wry2", 0},  {"l_leg_akx", 0},   {"l_leg_aky", 0},
  //{"l_leg_hpx", 0},   {"l_leg_hpy", 0},   {"l_leg_hpz", 0},
  //{"l_leg_kny", 0},   {"neck_ry", 0},     {"r_arm_elx", 0},
  //{"r_arm_ely", 0},   {"r_arm_shx", 0},   {"r_arm_shz", 0},
  //{"r_arm_wrx", 0},   {"r_arm_wry", 0},   {"r_arm_wry2", 0},
  //{"r_leg_akx", 0.1}, {"r_leg_aky", 0.2}, {"r_leg_hpx", 0},
  //{"r_leg_hpy", 0},   {"r_leg_hpz", 0},   {"r_leg_kny", 0}};
  // std::map<std::string, double> joint_vel = {
  //{"back_bkx", 0},    {"back_bky", 0},    {"back_bkz", 0},
  //{"l_arm_elx", 0},   {"l_arm_ely", 0},   {"l_arm_shx", 0},
  //{"l_arm_shz", 0},   {"l_arm_wrx", 0},   {"l_arm_wry", 0},
  //{"l_arm_wry2", 0},  {"l_leg_akx", 0},   {"l_leg_aky", 0},
  //{"l_leg_hpx", 0},   {"l_leg_hpy", 0},   {"l_leg_hpz", 0},
  //{"l_leg_kny", 0},   {"neck_ry", 0},     {"r_arm_elx", 0},
  //{"r_arm_ely", 0},   {"r_arm_shx", 0},   {"r_arm_shz", 0},
  //{"r_arm_wrx", 0},   {"r_arm_wry", 0},   {"r_arm_wry2", 0},
  //{"r_leg_akx", 0.1}, {"r_leg_aky", 0.2}, {"r_leg_hpx", 0},
  //{"r_leg_hpy", 0},   {"r_leg_hpz", 0},   {"r_leg_kny", 0}};

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

  pin_robot.UpdateRobotModel(com_pos, com_quat.normalized(), com_lv, com_av,
                             bjoint_pos, bjoint_quat.normalized(), bjoint_lv,
                             bjoint_av, joint_pos, joint_vel, true);

  std::cout << "============PINOCCHIO robot updated=============" << std::endl;

  std::cout << "r_sole pos with string: " << std::endl;

  auto tic = std::chrono::high_resolution_clock::now();
  Eigen::Isometry3d iso = pin_robot.GetLinkIso("r_foot_contact");
  auto toc = std::chrono::high_resolution_clock::now();

  std::cout << iso.linear() << std::endl;
  std::cout << iso.translation() << std::endl;
  std::cout << "time passed: "
            << std::chrono::duration<double, std::milli>(toc - tic).count()
            << "ms" << std::endl;

  std::cout << "r_sole pos with int: " << std::endl;
  int r_foot_contact(58);
  auto tic1 = std::chrono::high_resolution_clock::now();
  Eigen::Isometry3d iso2 = pin_robot.GetLinkIso(r_foot_contact);
  auto toc1 = std::chrono::high_resolution_clock::now();
  std::cout << iso2.linear() << std::endl;
  std::cout << iso2.translation() << std::endl;
  std::cout << "time passed: "
            << std::chrono::duration<double, std::milli>(toc1 - tic1).count()
            << "ms" << std::endl;

  // std::cout << "r_sole position: " << std::endl;
  // Eigen::Isometry3d iso = pin_robot->GetLinkIso("r_sole");
  // std::cout << iso.linear() << std::endl;
  // std::cout << iso.translation() << std::endl;

  // std::cout << "r_sole velocity" << std::endl;
  // Eigen::MatrixXd vel = pin_robot->GetLinkVel("r_sole");
  // std::cout << vel << std::endl;

  // std::cout << "Jacobian Matrix:" << std::endl;
  // Eigen::MatrixXd jacobian = pin_robot->GetLinkJacobian("r_sole");
  // std::cout << jacobian << std::endl;

  // std::cout << "base_local_com_pos:" << std::endl;
  // Eigen::Vector3d localcom = pin_robot->GetBaseLocalComPos();
  // std::cout << localcom << std::endl;

  // std::cout << "base_link_name:" << std::endl;
  // string bLinkName = pin_robot->GetBaseLinkName();
  // std::cout << bLinkName << std::endl;

  // std::cout << "q_idx:" << std::endl;
  // int q_idx = pin_robot->GetQIdx("back_bkx");
  // std::cout << q_idx << std::endl;

  // std::cout << "q_dot_idx:" << std::endl;
  // int q_d_idx = pin_robot->GetQdotIdx("back_bkx");
  // std::cout << q_d_idx << std::endl;

  // std::cout << "joint_idx:" << std::endl;
  // int j_idx = pin_robot->GetJointIdx("back_bkx");
  // std::cout << j_idx << std::endl;

  // std::cout << "q:" << std::endl;
  // Eigen::VectorXd _q = pin_robot->GetQ();
  // std::cout << _q << std::endl;

  // std::cout << "q_dot:" << std::endl;
  // Eigen::VectorXd _q_dot = pin_robot->GetQdot();
  // std::cout << _q_dot << std::endl;

  // std::cout << "mass matrix:" << std::endl;
  // Eigen::MatrixXd mass = pin_robot->GetMassMatrix();
  // std::cout << mass << std::endl;

  // std::cout << "gravity:" << std::endl;
  // Eigen::VectorXd gravity = pin_robot->GetGravity();
  // std::cout << gravity << std::endl;

  // std::cout << "coriolis:" << std::endl;
  // Eigen::VectorXd cori = pin_robot->GetCoriolis();
  // std::cout << cori << std::endl;

  // std::cout << "com pos:" << std::endl;
  // Eigen::Vector3d com = pin_robot->GetRobotComPos();
  // std::cout << com << std::endl;

  // std::cout << "com lin vel:" << std::endl;
  // Eigen::Vector3d comlv = pin_robot->GetRobotComLinVel();
  // std::cout << comlv << std::endl;

  // std::cout << "com jacobian:" << std::endl;
  // Eigen::MatrixXd comJ = pin_robot->GetComLinJacobian();
  // std::cout << comJ << std::endl;

  // std::cout << "com jacobian_dot:" << std::endl;
  // Eigen::MatrixXd comJd = pin_robot->GetComLinJacobianDot();
  // std::cout << comJd << std::endl;

  // std::cout << "link jacobian_dot times q_dot:" << std::endl;
  // Eigen::MatrixXd linkJ = pin_robot->GetLinkJacobianDotQdot("r_sole");
  // std::cout << linkJ << std::endl;

  // std::cout << "Ig" << std::endl;
  // std::cout << pin_robot->Ig_ << std::endl;

  // std::cout << "hg" << std::endl;
  // std::cout << pin_robot->Hg_ << std::endl;

  return 0;
}

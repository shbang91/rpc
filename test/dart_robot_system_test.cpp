#include <iostream>
#include <map>
#include <string>

#include "configuration.hpp"
#include "pnc/robot_system/dart_robot_system.hpp"

int main() {
  RobotSystem *dart_robot = new DartRobotSystem(
      THIS_COM "robot_model/atlas/atlas_rel_path.urdf", false, false);

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

  std::cout << "============DART robot updated=============" << std::endl;

  // std::cout << "r_sole position: " << std::endl;
  // Eigen::Isometry3d iso = dart_robot->GetLinkIso("r_sole");
  // std::cout << iso.linear() << std::endl;
  // std::cout << iso.translation() << std::endl;

  // std::cout << "r_sole velocity" << std::endl;
  // Eigen::MatrixXd vel = dart_robot->GetLinkVel("r_sole");
  // std::cout << vel << std::endl;

  std::cout << "Jacobian Matrix:" << std::endl;
  Eigen::MatrixXd jacobian = dart_robot->GetLinkJacobian("r_sole");
  std::cout << jacobian << std::endl;

  // std::cout << "base_local_com_pos:" << std::endl;
  // Eigen::Vector3d localcom = dart_robot->GetBaseLocalComPos();
  // std::cout << localcom << std::endl;

  // std::cout << "base_link_name:" << std::endl;
  // string bLinkName = dart_robot->GetBaseLinkName();
  // std::cout << bLinkName << std::endl;

  // std::cout << "q_idx:" << std::endl;
  // int q_idx = dart_robot->GetQIdx("back_bkx");
  // std::cout << q_idx << std::endl;

  // std::cout << "q_dot_idx:" << std::endl;
  // int q_d_idx = dart_robot->GetQdotIdx("back_bkx");
  // std::cout << q_d_idx << std::endl;

  // std::cout << "joint_idx:" << std::endl;
  // int j_idx = dart_robot->GetJointIdx("back_bkx");
  // std::cout << j_idx << std::endl;

  // std::cout << "q:" << std::endl;
  // Eigen::VectorXd _q = dart_robot->GetQ();
  // std::cout << _q << std::endl;

  // std::cout << "q_dot:" << std::endl;
  // Eigen::VectorXd _q_dot = dart_robot->GetQdot();
  // std::cout << _q_dot << std::endl;

  // std::cout << "mass matrix:" << std::endl;
  // Eigen::MatrixXd mass = dart_robot->GetMassMatrix();
  // std::cout << mass << std::endl;

  // std::cout << "gravity:" << std::endl;
  // Eigen::VectorXd gravity = dart_robot->GetGravity();
  // std::cout << gravity << std::endl;

  // std::cout << "coriolis:" << std::endl;
  // Eigen::VectorXd cori = dart_robot->GetCoriolis();
  // std::cout << cori << std::endl;

  // std::cout << "com pos:" << std::endl;
  // Eigen::Vector3d com = dart_robot->GetRobotComPos();
  // std::cout << com << std::endl;

  std::cout << "com lin vel:" << std::endl;
  Eigen::Vector3d comlv = dart_robot->GetRobotComLinVel();
  std::cout << comlv << std::endl;

  std::cout << "com jacobian:" << std::endl;
  Eigen::MatrixXd comJ = dart_robot->GetComLinJacobian();
  std::cout << comJ << std::endl;

  std::cout << "com jacobian_dot:" << std::endl;
  Eigen::MatrixXd comJd = dart_robot->GetComLinJacobianDot();
  std::cout << comJd << std::endl;

  std::cout << "link jacobian_dot times q_dot:" << std::endl;
  Eigen::MatrixXd linkJ = dart_robot->GetLinkJacobianDotQdot("r_sole");
  std::cout << linkJ << std::endl;

  std::cout << "Ig" << std::endl;
  std::cout << dart_robot->Ig_ << std::endl;

  std::cout << "hg" << std::endl;
  std::cout << dart_robot->Hg_ << std::endl;
  delete dart_robot;

  return 0;
}

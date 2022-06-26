#include <iostream>
#include <map>
#include <string>

#include "configuration.hpp"
#include "pnc/robot_system/dart_robot_system.hpp"

constexpr double PI = 3.141592653589793238463;

int main() {
  DartRobotSystem dart_robot(THIS_COM "robot_model/draco/draco_rel_path.urdf",
                             false, false);

  Eigen::Vector3d com_pos(0, 0, 1.5 - 0.757);
  Eigen::Vector3d com_lv(0.1, 0, 0);
  Eigen::Vector3d com_av(0.1, 0, 0);
  Eigen::Quaternion<double> com_quat(1, 0, 0, 0);
  Eigen::Vector3d bjoint_pos(0, 0, 1.5 - 0.757);
  Eigen::Vector3d bjoint_lv(0.1, 0, 0);
  Eigen::Vector3d bjoint_av(0.1, 0, 0);
  Eigen::Quaternion<double> bjoint_quat(0.7071, 0, 0, 0.7071);

  std::map<std::string, double> joint_pos = {{"l_hip_ie", 0},
                                             {"l_hip_aa", 0},
                                             {"l_hip_fe", -PI / 4},
                                             {"l_knee_fe_jp", PI / 4},
                                             {"l_knee_fe_jd", PI / 4},
                                             {"l_ankle_fe", -PI / 4},
                                             {"l_ankle_ie", 0},
                                             {"l_shoulder_fe", 0},
                                             {"l_shoulder_aa", PI / 6},
                                             {"l_shoulder_ie", 0},
                                             {"l_elbow_fe", -PI / 2},
                                             {"l_wrist_ps", 0},
                                             {"l_wrist_pitch", 0},
                                             {"neck_pitch", 0},
                                             {"r_hip_ie", 0},
                                             {"r_hip_aa", 0},
                                             {"r_hip_fe", -PI / 4},
                                             {"r_knee_fe_jp", PI / 4},
                                             {"r_knee_fe_jd", PI / 4},
                                             {"r_ankle_fe", -PI / 4},
                                             {"r_ankle_ie", 0},
                                             {"r_shoulder_fe", 0},
                                             {"r_shoulder_aa", -PI / 6},
                                             {"r_shoulder_ie", 0},
                                             {"r_elbow_fe", -PI / 2},
                                             {"r_wrist_ps", 0},
                                             {"r_wrist_pitch", 0}};
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

  dart_robot.UpdateRobotModel(com_pos, com_quat.normalized(), com_lv, com_av,
                              bjoint_pos, bjoint_quat.normalized(), bjoint_lv,
                              bjoint_av, joint_pos, joint_vel, true);

  std::cout << "============DART robot updated=============" << std::endl;
  // std::cout << "base_local_com_pos:" << std::endl;
  // Eigen::Vector3d localcom = dart_robot.GetBaseLocalComPos();
  // std::cout << localcom << std::endl;

  // std::cout << "base_link_name:" << std::endl;
  // string bLinkName = dart_robot.GetBaseLinkName();
  // std::cout << bLinkName << std::endl;

  // std::cout << "q_idx:" << std::endl;
  // int q_idx = dart_robot.GetQIdx("back_bkx");
  // std::cout << q_idx << std::endl;

  // std::cout << "q_dot_idx:" << std::endl;
  // int q_d_idx = dart_robot.GetQdotIdx("back_bkx");
  // std::cout << q_d_idx << std::endl;

  // std::cout << "joint_idx:" << std::endl;
  // int j_idx = dart_robot.GetJointIdx("back_bkx");
  // std::cout << j_idx << std::endl;

  // std::cout << "q:" << std::endl;
  // Eigen::VectorXd _q = dart_robot.GetQ();
  // std::cout << _q << std::endl;

  // std::cout << "q_dot:" << std::endl;
  // Eigen::VectorXd _q_dot = dart_robot.GetQdot();
  // std::cout << _q_dot << std::endl;

  std::cout << "=========================================" << std::endl;
  std::cout << "mass matrix:" << std::endl;
  Eigen::MatrixXd mass = dart_robot.GetMassMatrix();
  Eigen::MatrixXd modified_mass = mass;
  modified_mass.leftCols(3) = mass.middleCols(3, 3);
  modified_mass.middleCols(3, 3) = mass.leftCols(3);
  Eigen::MatrixXd temp_mat = modified_mass.topRows(3);
  modified_mass.topRows(3) = modified_mass.middleRows(3, 3);
  modified_mass.middleRows(3, 3) = temp_mat;
  std::cout << modified_mass << std::endl;

  // std::cout << "gravity:" << std::endl;
  // Eigen::VectorXd gravity = dart_robot.GetGravity();
  // std::cout << gravity << std::endl;

  // std::cout << "coriolis:" << std::endl;
  // Eigen::VectorXd cori = dart_robot.GetCoriolis();
  // std::cout << cori << std::endl;

  // std::cout << "com pos:" << std::endl;
  // Eigen::Vector3d com = dart_robot.GetRobotComPos();
  // std::cout << com << std::endl;

  // std::cout << "com lin vel:" << std::endl;
  // Eigen::Vector3d comlv = dart_robot.GetRobotComLinVel();
  // std::cout << comlv << std::endl;

  std::cout << "=========================================" << std::endl;
  std::cout << "com jacobian:" << std::endl;
  Eigen::Matrix<double, 3, Eigen::Dynamic> comJ =
      dart_robot.GetComLinJacobian();
  Eigen::Matrix<double, 3, Eigen::Dynamic> mod_J = comJ;
  mod_J.leftCols(3) = comJ.middleCols(3, 3);
  mod_J.middleCols(3, 3) = comJ.leftCols(3);
  std::cout << mod_J << std::endl;

  std::cout << "=========================================" << std::endl;
  std::cout << "com jdot_qdot:" << std::endl;
  Eigen::MatrixXd comJd = dart_robot.GetComLinJacobianDot();
  Eigen::VectorXd qdot = dart_robot.GetQdot();
  std::cout << comJd * qdot.transpose() << std::endl;

  // std::cout << "Ig" << std::endl;
  // std::cout << dart_robot.Ig_ << std::endl;

  // std::cout << "hg" << std::endl;
  // std::cout << dart_robot.Hg_ << std::endl;

  // std::cout << "Ag" << std::endl;
  // std::cout << dart_robot.Ag_ << std::endl;

  // std::cout << "l_foot_contact position: " << std::endl;
  // Eigen::Isometry3d iso = dart_robot.GetLinkIso("r_foot_contact");
  // std::cout << iso.linear() << std::endl;
  // std::cout << iso.translation() << std::endl;

  // std::cout << "l_foot_contact velocity" << std::endl;
  // Eigen::Matrix<double, 6, 1> vel = dart_robot.GetLinkVel("r_foot_contact");
  // std::cout << vel << std::endl;

  std::cout << "=========================================" << std::endl;
  std::cout << "Jacobian Matrix:" << std::endl;
  Eigen::Matrix<double, 6, Eigen::Dynamic> jacobian =
      dart_robot.GetLinkJacobian("r_foot_contact");
  Eigen::Matrix<double, 6, Eigen::Dynamic> modified_jac = jacobian;
  modified_jac.leftCols(3) = jacobian.middleCols(3, 3);
  modified_jac.middleCols(3, 3) = jacobian.leftCols(3);
  std::cout << modified_jac << std::endl;

  return 0;
}

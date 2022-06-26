#include <chrono>
#include <iostream>
#include <string>

#include "configuration.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/draco_controller/draco_definition.hpp"

constexpr double PI = 3.141592653589793238463;
int main() {

  PinocchioRobotSystem robot(THIS_COM "robot_model/draco/draco_modified.urdf",
                             THIS_COM "robot_model/draco", false, false);

  Eigen::Vector3d bjoint_pos(0, 0, 1.5 - 0.757);
  Eigen::Quaterniond bjoint_quat(0.7071, 0, 0, 0.7071);
  Eigen::Vector3d bjoint_lv(0.1, 0, 0);
  Eigen::Vector3d bjoint_av(0.1, 0, 0);

  Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(draco::n_adof);
  Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(draco::n_adof);
  joint_pos[draco_joint::l_hip_fe] = -PI / 4;
  joint_pos[draco_joint::l_knee_fe_jp] = PI / 4;
  joint_pos[draco_joint::l_knee_fe_jd] = PI / 4;
  joint_pos[draco_joint::l_ankle_fe] = -PI / 4;
  joint_pos[draco_joint::l_shoulder_aa] = PI / 6;
  joint_pos[draco_joint::l_elbow_fe] = -PI / 2;

  joint_pos[draco_joint::r_hip_fe] = -PI / 4;
  joint_pos[draco_joint::r_knee_fe_jp] = PI / 4;
  joint_pos[draco_joint::r_knee_fe_jd] = PI / 4;
  joint_pos[draco_joint::r_ankle_fe] = -PI / 4;
  joint_pos[draco_joint::r_shoulder_aa] = -PI / 6;
  joint_pos[draco_joint::r_elbow_fe] = -PI / 2;

  robot.UpdateRobotModel(bjoint_pos, bjoint_quat.normalized(), bjoint_lv,
                         bjoint_av, joint_pos, joint_vel, true);

  std::cout << "============PINOCCHIO robot updated=============" << std::endl;

  std::cout << "=========================================" << std::endl;
  std::cout << "mass matrix" << std::endl;
  std::cout << robot.GetMassMatrix() << std::endl;

  // std::cout << "gravity" << std::endl;
  // std::cout << robot.GetGravity() << std::endl;

  // std::cout << "coriolis" << std::endl;
  // std::cout << robot.GetCoriolis() << std::endl;
  // std::cout << "com" << std::endl;
  // std::cout << robot.GetRobotComPos() << std::endl;

  // std::cout << "com vel" << std::endl;
  // std::cout << robot.GetRobotComLinVel() << std::endl;

  std::cout << "=========================================" << std::endl;
  std::cout << "com jac" << std::endl;
  std::cout << robot.GetComLinJacobian() << std::endl;

  std::cout << "=========================================" << std::endl;
  std::cout << "com jdot_qdot" << std::endl;
  std::cout << robot.GetComLinJacobianDotQdot().transpose() << std::endl;

  // std::cout << "Ig" << std::endl;
  // std::cout << robot.GetIg() << std::endl;

  // std::cout << "Hg" << std::endl;
  // std::cout << robot.GetHg() << std::endl;

  // std::cout << "Ag" << std::endl;
  // std::cout << robot.GetAg() << std::endl;

  // std::cout << "rfoot pos" << std::endl;
  // Eigen::Isometry3d rfoot_iso =
  // robot.GetLinkIsometry(draco_link::r_foot_contact);
  // std::cout << rfoot_iso.translation() << std::endl;
  // std::cout << rfoot_iso.linear() << std::endl;

  // std::cout << "rfoot vel" << std::endl;
  // Eigen::Matrix<double, 6, 1> vel =
  // robot.GetLinkSpatialVel(draco_link::r_foot_contact);
  // std::cout << vel.transpose() << std::endl;

  std::cout << "=========================================" << std::endl;
  std::cout << "rfoot jac" << std::endl;
  std::cout << robot.GetLinkJacobian(draco_link::r_foot_contact) << std::endl;

  // std::cout << "rfoot link acc drift" << std::endl;
  // std::cout << robot
  //.GetLinkJacobianDotQdot(draco_link::r_foot_contact,
  // pinocchio::LOCAL_WORLD_ALIGNED)
  //.transpose()
  //<< std::endl;

  return 0;
}

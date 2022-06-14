#include <chrono>
#include <iostream>
#include <string>

#include "configuration.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "pnc/draco_pnc/draco_definition.hpp"

int main() {

  PinocchioRobotSystem robot(THIS_COM "robot_model/draco/draco_modified.urdf",
                             THIS_COM "robot_model/draco", false, false);

  // Eigen::Vector3d com_pos(0, 0, 0);
  // Eigen::Vector3d com_lv(0.1, 0, 0);
  // Eigen::Vector3d com_av(0.1, 0, 0);
  // Eigen::Quaternion<double> com_quat(1, 0, 0, 0);
  Eigen::Vector3d bjoint_pos(0, 0, 0);
  Eigen::Quaterniond bjoint_quat(1, 0, 0, 0);
  Eigen::Vector3d bjoint_lv(0.1, 0, 0);
  Eigen::Vector3d bjoint_av(0.1, 0, 0);

  Eigen::VectorXd joint_pos = Eigen::VectorXd::Zero(draco::n_adof);
  Eigen::VectorXd joint_vel = Eigen::VectorXd::Zero(draco::n_adof);

  robot.UpdateRobotModel(bjoint_pos, bjoint_quat.normalized(), bjoint_lv,
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
  // Eigen::Isometry3d rfoot_iso =
  // robot.GetLinkIsometry(draco_link::r_foot_contact);
  // std::cout << rfoot_iso.translation() << std::endl;
  // std::cout << rfoot_iso.linear() << std::endl;

  // std::cout << "rfoot vel" << std::endl;
  // Eigen::Matrix<double, 6, 1> vel = robot.GetLinkSpatialVel(
  // draco_link::r_foot_contact, pinocchio::LOCAL_WORLD_ALIGNED);
  // std::cout << vel.transpose() << std::endl;

  // std::cout << "rfoot jac" << std::endl;
  // std::cout << robot.GetLinkJacobian(draco_link::r_foot_contact,
  // pinocchio::LOCAL_WORLD_ALIGNED)
  //<< std::endl;

  // std::cout << "rfoot link acc drift" << std::endl;
  // std::cout << robot
  //.GetLinkJacobianDotQdot(draco_link::r_foot_contact,
  // pinocchio::LOCAL_WORLD_ALIGNED)
  //.transpose()
  //<< std::endl;

  // std::cout << "com" << std::endl;
  // std::cout << robot.GetRobotComPos() << std::endl;

  // std::cout << "com vel" << std::endl;
  // std::cout << robot.GetRobotComLinVel() << std::endl;

  // std::cout << "com jac" << std::endl;
  // std::cout << robot.GetComLinJacobian() << std::endl;

  // std::cout << "com jdot_qdot" << std::endl;
  // std::cout << robot.GetComLinJacobianDotQdot().transpose() << std::endl;

  return 0;
}

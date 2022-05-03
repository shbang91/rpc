#pragma once

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/utils/utils.hpp>

#include "pnc/robot_system/robot_system.hpp"

/*
 *  Dart considers floating base with 6 positions and 6 velocities with the
 *  order of rotx, roty, rotz, x, y, z. Therefore, n_q_ = n_qdot_
 *  Note that the joint named with 'rootJoint' has 6 dof to represent the
 *  floating base.
 */

class DartRobotSystem : public RobotSystem {
public:
  DartRobotSystem(const std::string &_urdf_path, const bool &_b_fixed_base,
                  const bool &_b_print_info);
  DartRobotSystem(dart::dynamics::SkeletonPtr _robot, const bool &_b_fixed_base,
                  const bool &_b_print_info);
  ~DartRobotSystem();

  // Configure the following properties: n_floating, n_q, n_q_dot, n_a,
  // total_mass, joint_pos_limit, joint_vel_limit, joint_trq_limit.
  virtual void ConfigRobot();

  virtual int GetQIdx(const std::string &_joint_name);
  virtual int GetQdotIdx(const std::string &_joint_name);
  virtual int GetJointIdx(const std::string &_joint_name);

  virtual std::map<std::string, double>
  EigenVectorToMap(const Eigen::VectorXd &_joint_cmd);
  virtual Eigen::VectorXd
  MapToEigenVector(std::map<std::string, double> _joint_map);

  virtual Eigen::Vector3d GetBaseLocalComPos();
  virtual std::string GetBaseLinkName();

  virtual void
  UpdateRobotModel(const Eigen::Vector3d &_base_com_pos,
                   const Eigen::Quaternion<double> &_base_com_quat,
                   const Eigen::Vector3d &_base_com_lin_vel,
                   const Eigen::Vector3d &_base_com_ang_vel,
                   const Eigen::Vector3d &_base_joint_pos,
                   const Eigen::Quaternion<double> &_base_joint_quat,
                   const Eigen::Vector3d &_base_joint_lin_vel,
                   const Eigen::Vector3d &_base_joint_ang_vel,
                   std::map<std::string, double> _joint_positions,
                   std::map<std::string, double> _joint_velocities,
                   const bool _b_update_centroid = false);

  // Update centroid quantities (Ig_, Ag_, Hg_)
  virtual void UpdateCentroidalQuantities();

  virtual Eigen::VectorXd GetQ();
  virtual Eigen::VectorXd GetQdot();
  virtual Eigen::MatrixXd GetInertiaMatrix();
  virtual Eigen::VectorXd GetGravity();
  virtual Eigen::VectorXd GetCoriolis();
  virtual Eigen::Vector3d GetRobotComPos();
  virtual Eigen::Vector3d GetRobotComLinVel();

  virtual Eigen::Isometry3d GetLinkIso(const std::string &_link_name);
  virtual Eigen::Matrix<double, 6, 1> GetLinkVel(const std::string &_link_name);
  virtual Eigen::Matrix<double, 6, Eigen::Dynamic>
  GetLinkJacobian(const std::string &_link_name);
  virtual Eigen::Matrix<double, 6, 1>
  GetLinkJacobianDotQdot(const std::string &_link_name);

  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> GetComLinJacobian();
  virtual Eigen::Matrix<double, 3, Eigen::Dynamic> GetComLinJacobianDot();

private:
  dart::dynamics::SkeletonPtr skeleton_;
  std::map<std::string, dart::dynamics::JointPtr> joint_ptr_map_;
  std::map<std::string, dart::dynamics::BodyNodePtr> body_node_ptr_map_;
};

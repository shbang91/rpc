#pragma once

#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Dense>
#include <map>
#include <string>

/*
 *  Pinnochio considers floating base with 7 positions and 6 velocities with the
 *  order of [x, y, z, quat_x, quat_y, quat_z, quat_w, joints] and
 *  [xdot, ydot, zdot, ang_x, ang_y, ang_z, joints].
 *  Note that first six element of generalized velocities are represented in the
 *  base joint frame acting on the base joint frame.
 */

class PinocchioRobotSystem {
public:
  PinocchioRobotSystem(const std::string &urdf_file,
                       const std::string &package_dir, const bool b_fixed_base,
                       const bool b_print_info);
  virtual ~PinocchioRobotSystem() = default;

  void UpdateRobotModel(const Eigen::Vector3d &base_joint_pos,
                        const Eigen::Quaterniond &base_joint_quat,
                        const Eigen::Vector3d &base_joint_lin_vel,
                        const Eigen::Vector3d &base_joint_ang_vel,
                        const Eigen::VectorXd &joint_pos,
                        const Eigen::VectorXd &joint_vel,
                        bool b_update_centroid = false);

  // kinematics getter
  Eigen::VectorXd GetQ() const;
  Eigen::VectorXd GetQdot() const;

  int GetQIdx(const int joint_idx) const;
  int GetQdotIdx(const int joint_idx) const;

  Eigen::VectorXd GetJointPos() const;
  Eigen::VectorXd GetJointVel() const;

  Eigen::Isometry3d GetLinkIsometry(const int link_idx);
  Eigen::Matrix<double, 6, 1> GetLinkSpatialVel(const int link_idx) const;

  Eigen::Matrix<double, 6, Eigen::Dynamic> GetLinkJacobian(const int link_idx);
  Eigen::Matrix<double, 6, 1> GetLinkJacobianDotQdot(const int link_idx);

  Eigen::Matrix<double, 6, 1> GetLinkBodySpatialVel(const int link_idx) const;

  Eigen::Matrix<double, 6, Eigen::Dynamic>
  GetLinkBodyJacobian(const int link_idx);
  Eigen::Matrix<double, 6, 1> GetLinkBodyJacobianDotQdot(const int link_idx);

  Eigen::Vector3d GetRobotComPos();
  Eigen::Vector3d GetRobotComLinVel();
  Eigen::Matrix<double, 3, Eigen::Dynamic> GetComLinJacobian();
  Eigen::Matrix<double, 3, 1> GetComLinJacobianDotQdot();

  // dynamics getter
  Eigen::MatrixXd GetMassMatrix();
  Eigen::VectorXd GetGravity();
  Eigen::VectorXd GetCoriolis();
  double GetTotalMass() const { return total_mass_; }

  // centroidal quantity getter
  Eigen::Matrix<double, 6, 6> GetIg() const;
  Eigen::Matrix<double, 6, 1> GetHg() const;
  Eigen::Matrix<double, 6, Eigen::Dynamic> GetAg() const;

  // limits
  Eigen::Matrix<double, Eigen::Dynamic, 2> joint_pos_limits_;
  Eigen::Matrix<double, Eigen::Dynamic, 2> joint_vel_limits_;
  Eigen::Matrix<double, Eigen::Dynamic, 2> joint_trq_limits_;

  // getter function
  int NumQdot() const;
  int NumActiveDof() const;
  int NumFloatDof() const;

  Eigen::Matrix<double, Eigen::Dynamic, 2> TrqLimit() const {
    return joint_trq_limits_;
  }

private:
  void _Initialize();
  void _UpdateCentroidalQuantities();
  void _PrintRobotInfo();

  pinocchio::Model model_;
  pinocchio::GeometryModel collision_model_;
  pinocchio::GeometryModel visual_model_;

  pinocchio::Data data_;
  pinocchio::GeometryData collision_data_;
  pinocchio::GeometryData visual_data_;

  // generalized coordinate configuration
  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;

  // centroidal quantities
  Eigen::Matrix<double, 6, 6> Ig_;
  Eigen::Matrix<double, 6, 1> Hg_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> Ag_;

  std::string urdf_file_;
  std::string package_dir_;
  bool b_fixed_base_;
  bool b_print_info_;

  int n_q_;
  int n_qdot_;
  int n_adof_;
  int n_float_;

  std::map<double, std::string> joint_idx_map_;
  std::map<double, std::string> link_idx_map_;

  double total_mass_;
};

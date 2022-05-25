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

#include "pnc/robot_system/robot_system.hpp"

/*
 *  Pinnochio considers floating base with 7 positions and 6 velocities with the
 *  order of [x, y, z, quat_x, quat_y, quat_z, quat_w, joints] and
 *  [xdot, ydot, zdot, ang_x, ang_y, ang_z, joints].
 *  Note that first six element of generalized velocities are represented in the
 *  base joint frame acting on the base joint frame.
 */

/// class PinocchioRobotSystem
class PinocchioRobotSystem : public RobotSystem {
public:
  /// \{ \name Constructor and Destructor
  PinocchioRobotSystem(const std::string &_urdf_file,
                       const std::string &_package_dir,
                       const bool _b_fixed_base,
                       const bool _b_print_info = false,
                       const int _num_virtual_dof = 0);
  ~PinocchioRobotSystem() = default;
  /// \}

  int GetQIdx(const std::string &joint_name) override;
  int GetQdotIdx(const std::string &joint_name) override;
  int GetJointIdx(const std::string &joint_name) override;

  std::map<std::string, double>
  EigenVectorToMap(const Eigen::VectorXd &_joint_cmd) override;
  Eigen::VectorXd MapToEigenVector(std::map<std::string, double>) override;
  Eigen::Vector3d GetBaseLocalComPos() override;
  std::string GetBaseLinkName() override;
  void UpdateRobotModel(const Eigen::Vector3d &base_com_pos,
                        const Eigen::Quaternion<double> &base_com_quat,
                        const Eigen::Vector3d &base_com_lin_vel,
                        const Eigen::Vector3d &base_com_ang_vel,
                        const Eigen::Vector3d &base_joint_pos,
                        const Eigen::Quaternion<double> &base_joint_quat,
                        const Eigen::Vector3d &base_joint_lin_vel,
                        const Eigen::Vector3d &base_joint_ang_vel,
                        const std::map<std::string, double> joint_pos,
                        const std::map<std::string, double> joint_vel,
                        const bool _b_update_centroid = false) override;
  Eigen::VectorXd GetQ() const override;
  Eigen::VectorXd GetQdot() const override;
  Eigen::MatrixXd GetMassMatrix() override;
  Eigen::VectorXd GetGravity() override;
  Eigen::VectorXd GetCoriolis() override;
  Eigen::Vector3d GetRobotComPos() override;
  Eigen::Vector3d GetRobotComLinVel() override;
  Eigen::Matrix<double, 3, Eigen::Dynamic> GetComLinJacobian() override;
  Eigen::Matrix<double, 3, Eigen::Dynamic> GetComLinJacobianDot() override;

  Eigen::Isometry3d GetLinkIso(const std::string &link_name) override;
  Eigen::Matrix<double, 6, 1> GetLinkVel(const std::string &link_name) override;
  Eigen::Matrix<double, 6, Eigen::Dynamic>
  GetLinkJacobian(const std::string &link_name) override;
  Eigen::Matrix<double, 6, 1>
  GetLinkJacobianDotQdot(const std::string &link_name) override;

  Eigen::Isometry3d GetLinkIso(const int &link_id);
  Eigen::Matrix<double, 6, 1>
  GetLinkVel(const int &link_id,
             const pinocchio::ReferenceFrame referenceframe =
                 pinocchio::LOCAL_WORLD_ALIGNED);
  Eigen::Matrix<double, 6, Eigen::Dynamic>
  GetLinkJacobian(const int &link_id,
                  const pinocchio::ReferenceFrame referenceframe =
                      pinocchio::LOCAL_WORLD_ALIGNED);
  Eigen::Matrix<double, 6, 1>
  GetLinkJacobianDotQdot(const int &link_id,
                         const pinocchio::ReferenceFrame referenceframe =
                             pinocchio::LOCAL_WORLD_ALIGNED);

private:
  void UpdateCentroidalQuantities() override;
  void ConfigRobot() override;
  void PrintRobotInfo();

  pinocchio::Model model_;
  pinocchio::GeometryModel collision_model_;
  pinocchio::GeometryModel visual_model_;

  pinocchio::Data data_;
  pinocchio::GeometryData collision_data_;
  pinocchio::GeometryData visual_data_;

  Eigen::VectorXd q_;
  Eigen::VectorXd qdot_;

  std::string urdf_file_;
  std::string package_dir_;
  int n_vdof_;

  /// Map of joint name and joint idx
  std::map<std::string, int> joint_id_;

  /// Map of link name and link idx
  std::map<std::string, int> link_id_;
};

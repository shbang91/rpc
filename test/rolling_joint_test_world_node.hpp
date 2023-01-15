#pragma once

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/gui/GLFuncs.hpp>
#include <dart/gui/osg/osg.hpp>

#include "controller/robot_system/dart_robot_system.hpp"

// class Interface;
// class AtlasSensorData;
// class AtlasCommand;

class RollingJointWorldNode : public dart::gui::osg::WorldNode {
private:
  // void GetBaseData(Eigen::Vector3d &_base_com_pos,
  // Eigen::Vector4d &_base_com_quat,
  // Eigen::Vector3d &_base_com_lin_vel,
  // Eigen::Vector3d &_base_com_ang_vel,
  // Eigen::Vector3d &_base_joint_pos,
  // Eigen::Vector4d &_base_joint_quat,
  // Eigen::Vector3d &_base_joint_lin_vel,
  // Eigen::Vector3d &_base_joint_ang_vel);
  // void GetJointData(std::map<std::string, double> &_joint_positions,
  // std::map<std::string, double> &_joint_velocities);
  // void GetContactSwitchData(bool &rfoot_contact, bool &lfoot_contact);
  // void SetParams();

  // Interface *interface_;
  // AtlasSensorData *sensor_data_;
  // AtlasCommand *command_;

  dart::simulation::WorldPtr world_;
  dart::dynamics::SkeletonPtr robot_;

  RobotSystem *robot_model_;

  int count_;
  double t_;
  double servo_dt_;
  int n_dof_;
  // double kp_;
  // double kd_;

public:
  RollingJointWorldNode(const dart::simulation::WorldPtr &world);
  virtual ~RollingJointWorldNode();

  void customPreStep() override;
};

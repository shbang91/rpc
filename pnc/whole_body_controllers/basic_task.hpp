#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

#include "pnc/robot_system/robot_system.hpp"
#include "pnc/whole_body_controllers/task.hpp"

class JointTask : public Task {
public:
  JointTask(RobotSystem *_robot);
  virtual ~JointTask();

  void UpdateOscCommand() override;

  void UpdateTaskJacobian() override;
  void UpdateTaskJacobianDotQdot() override;
};

class SelectedJointTask : public Task {
public:
  SelectedJointTask(RobotSystem *_robot,
                    const std::vector<std::string> &_joint_container);
  virtual ~SelectedJointTask();

  void UpdateOscCommand() override;

  void UpdateTaskJacobian() override;
  void UpdateTaskJacobianDotQdot() override;

private:
  std::vector<std::string> joint_container_;
};

class LinkPosTask : public Task {
public:
  LinkPosTask(RobotSystem *_robot, const std::string &_target_link);
  virtual ~LinkPosTask();

  void UpdateOscCommand() override;

  void UpdateTaskJacobian() override;
  void UpdateTaskJacobianDotQdot() override;

private:
  std::string target_link_;
};

class LinkOriTask : public Task {
public:
  LinkOriTask(RobotSystem *_robot, const std::string &_target_link);
  virtual ~LinkOriTask();

  void UpdateOscCommand() override;

  void UpdateTaskJacobian() override;
  void UpdateTaskJacobianDotQdot() override;

private:
  std::string target_link_;
};

class ComTask : public Task {
public:
  ComTask(RobotSystem *_robot);
  virtual ~ComTask();

  void UpdateOscCommand() override;

  void UpdateTaskJacobian() override;
  void UpdateTaskJacobianDotQdot() override;
};

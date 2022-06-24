#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

#include "controller/whole_body_controller/task.hpp"
#include "util/util.hpp"

class JointTask : public Task {
public:
  JointTask(PinocchioRobotSystem *robot);
  virtual ~JointTask() = default;

  void UpdateOscCommand() override;

  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;
};

class SelectedJointTask : public Task {
public:
  SelectedJointTask(PinocchioRobotSystem *robot,
                    const std::vector<int> &joint_idx_container);
  virtual ~SelectedJointTask() = default;

  void UpdateOscCommand() override;

  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;

  std::vector<int> JointIdxContainer();

private:
  std::vector<int> joint_idx_container_;
};

class LinkPosTask : public Task {
public:
  LinkPosTask(PinocchioRobotSystem *robot, int target_link);
  virtual ~LinkPosTask() = default;

  void UpdateOscCommand() override;

  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;

private:
  int target_link_idx_;
};

class LinkOriTask : public Task {
public:
  LinkOriTask(PinocchioRobotSystem *robot, int target_link);
  virtual ~LinkOriTask() = default;

  void UpdateOscCommand() override;

  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;

private:
  int target_link_idx_;
};

class ComTask : public Task {
public:
  ComTask(PinocchioRobotSystem *robot);
  virtual ~ComTask() = default;

  void UpdateOscCommand() override;

  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;
};

#pragma once
#include <Eigen/Dense>
#include <iostream>

#include "configuration.hpp"

class FootStep {
public:
  FootStep();
  FootStep(const Eigen::Vector3d &init_pos, const Eigen::Quaterniond &init_quat,
           const int init_foot_side);
  virtual ~FootStep() = default;

  void ComputeMidFoot(const FootStep &footstep1, const FootStep &footstep2,
                      FootStep &midfoot);
  void PrintInfo();

  // setter
  void SetPosOriSide(const Eigen::Vector3d &pos, const Eigen::Quaterniond &quat,
                     const int foot_side) {
    pos_ = pos;
    quat_ori_ = quat;
    rot_ori_ = quat.toRotationMatrix();
    foot_side_ = foot_side;
  }

  void SetPosOri(const Eigen::Vector3d &pos, Eigen::Quaterniond &quat) {
    pos_ = pos;
    quat_ori_ = quat;
    rot_ori_ = quat.toRotationMatrix();
  }

  void SetRightSide() { foot_side_ = end_effector::RFoot; }
  void SetLeftSide() { foot_side_ = end_effector::LFoot; }
  void SetMidFoot() { foot_side_ = end_effector::MidFootType; }

  // getter
  int GetFootSide() const { return foot_side_; }
  Eigen::Vector3d GetPos() const { return pos_; }
  Eigen::Quaterniond GetOrientation() const { return quat_ori_; }
  Eigen::Matrix3d GetRotMat() const { return rot_ori_; }

private:
  Eigen::Vector3d pos_;
  Eigen::Quaterniond quat_ori_;
  Eigen::Matrix3d rot_ori_;

  int foot_side_;
};

#pragma once
#include <Eigen/Dense>
#include <iostream>
#include <vector>

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

  void SetPosOri(const Eigen::Vector3d &pos, const Eigen::Quaterniond &quat) {
    pos_ = pos;
    quat_ori_ = quat;
    rot_ori_ = quat.toRotationMatrix();
  }

  void SetPos(const Eigen::Vector3d &pos) { pos_ = pos; }

  void SetOri(const Eigen::Quaterniond &quat) {
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

  // Footstep generation static method for utility function
  static std::vector<FootStep>
  GetFwdWalkFootStep(const int n_steps, const double forward_step_length,
                     const double nominal_footwidth, const int first_swing_leg,
                     const FootStep &current_mid_foot);

  static std::vector<FootStep>
  GetInPlaceWalkFootStep(const int n_steps, const double nominal_footwidth,
                         const int first_swing_leg,
                         const FootStep &current_mid_foot);

  static std::vector<FootStep>
  GetTurningFootStep(const int n_steps, const double turn_radians_per_step,
                     const double nominal_footwidth,
                     const FootStep &current_mid_foot);

  static std::vector<FootStep>
  GetStrafeFootStep(const int n_steps, const double strafe_distance,
                    const double nominal_footwidth,
                    const FootStep &current_mid_foot);

private:
  Eigen::Vector3d pos_;
  Eigen::Quaterniond quat_ori_;
  Eigen::Matrix3d rot_ori_;

  int foot_side_;
};

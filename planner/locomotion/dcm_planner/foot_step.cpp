#include "planner/locomotion/dcm_planner/foot_step.hpp"

FootStep::FootStep()
    : pos_(Eigen::Vector3d::Zero()), quat_ori_(Eigen::Quaterniond::Identity()),
      rot_ori_(Eigen::Matrix3d::Identity()),
      foot_side_(end_effector::MidFootType) {}

FootStep::FootStep(const Eigen::Vector3d &init_pos,
                   const Eigen::Quaterniond &init_quat,
                   const int init_foot_side) {

  pos_ = init_pos;
  quat_ori_ = init_quat;
  rot_ori_ = init_quat.toRotationMatrix();
  foot_side_ = init_foot_side;
}

void FootStep::ComputeMidFoot(const FootStep &footstep1,
                              const FootStep &footstep2, FootStep &midfoot) {
  Eigen::Vector3d mid_pos = 0.5 * (footstep1.GetPos() + footstep2.GetPos());
  Eigen::Quaterniond mid_quat =
      footstep1.GetOrientation().slerp(0.5, footstep2.GetOrientation());
  midfoot.SetPosOri(mid_pos, mid_quat);
  midfoot.SetMidFoot();
}

void FootStep::PrintInfo() {
  if (foot_side_ == end_effector::LFoot || foot_side_ == end_effector::RFoot) {
    std::cout << "foot side = "
              << (foot_side_ == end_effector::LFoot ? "end_effector::LFoot"
                                                    : "end_effector::RFoot")
              << std::endl;
  } else if (foot_side_ == end_effector::MidFootType) {
    std::cout << "foot side = "
              << "end_effector::MidFootType" << std::endl;
  }

  std::cout << "pos = " << pos_.transpose() << std::endl;
  std::cout << "ori = " << quat_ori_.coeffs().transpose() << std::endl;
}

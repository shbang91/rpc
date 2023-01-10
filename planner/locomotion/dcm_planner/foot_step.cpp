#include "planner/locomotion/dcm_planner/foot_step.hpp"
#include "util/util.hpp"

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

std::vector<FootStep> FootStep::GetFwdWalkFootStep(
    const int n_steps, const double forward_step_length,
    const double nominal_footwidth, const int first_swing_leg,
    const FootStep &current_mid_foot) {
  std::vector<FootStep> foot_step_list;

  FootStep new_foot_step;

  int swing_leg_side = first_swing_leg;
  for (int i(0); i < n_steps; ++i) {
    if (swing_leg_side == end_effector::LFoot) {
      Eigen::Vector3d local_offset(static_cast<double>(i + 1) *
                                       forward_step_length,
                                   nominal_footwidth / 2., 0.);
      new_foot_step.SetPosOriSide(
          current_mid_foot.GetPos() +
              current_mid_foot.GetRotMat() * local_offset,
          current_mid_foot.GetOrientation(), end_effector::LFoot);
      swing_leg_side = end_effector::RFoot;
    } else {
      Eigen::Vector3d local_offset(static_cast<double>(i + 1) *
                                       forward_step_length,
                                   -nominal_footwidth / 2., 0.);
      new_foot_step.SetPosOriSide(
          current_mid_foot.GetPos() +
              current_mid_foot.GetRotMat() * local_offset,
          current_mid_foot.GetOrientation(), end_effector::RFoot);
      swing_leg_side = end_effector::LFoot;
    }
    foot_step_list.push_back(new_foot_step);
  }

  // add additional step forward to square the feet
  if (swing_leg_side == end_effector::LFoot) {
    Eigen::Vector3d local_offset(static_cast<double>(n_steps) *
                                     forward_step_length,
                                 nominal_footwidth / 2., 0.);
    new_foot_step.SetPosOriSide(
        current_mid_foot.GetPos() + current_mid_foot.GetRotMat() * local_offset,
        current_mid_foot.GetOrientation(), end_effector::LFoot);
  } else {
    Eigen::Vector3d local_offset(static_cast<double>(n_steps) *
                                     forward_step_length,
                                 -nominal_footwidth / 2., 0.);
    new_foot_step.SetPosOriSide(
        current_mid_foot.GetPos() + current_mid_foot.GetRotMat() * local_offset,
        current_mid_foot.GetOrientation(), end_effector::RFoot);
  }
  foot_step_list.push_back(new_foot_step);

  return foot_step_list;
}

std::vector<FootStep> FootStep::GetInPlaceWalkFootStep(
    const int n_steps, const double nominal_footwidth,
    const int first_swing_leg, const FootStep &current_mid_foot) {
  std::vector<FootStep> foot_step_list;

  FootStep new_foot_step;

  int swing_leg_side = first_swing_leg;
  for (int i(0); i < n_steps; ++i) {
    if (swing_leg_side == end_effector::LFoot) {
      Eigen::Vector3d local_offset(0., nominal_footwidth / 2., 0.);
      new_foot_step.SetPosOriSide(
          current_mid_foot.GetPos() +
              current_mid_foot.GetRotMat() * local_offset,
          current_mid_foot.GetOrientation(), end_effector::LFoot);
      swing_leg_side = end_effector::RFoot;
    } else {
      Eigen::Vector3d local_offset(0., -nominal_footwidth / 2., 0.);
      new_foot_step.SetPosOriSide(
          current_mid_foot.GetPos() +
              current_mid_foot.GetRotMat() * local_offset,
          current_mid_foot.GetOrientation(), end_effector::RFoot);
      swing_leg_side = end_effector::LFoot;
    }
    foot_step_list.push_back(new_foot_step);
  }
  return foot_step_list;
}

std::vector<FootStep> FootStep::GetTurningFootStep(
    const int n_steps, const double turn_radians_per_step,
    const double nominal_footwidth, const FootStep &current_mid_foot) {
  std::vector<FootStep> foot_step_list;

  FootStep new_right_foot, new_left_foot;
  FootStep rotated_mid_foot = current_mid_foot;

  Eigen::Quaterniond quat_increment_local(
      Eigen::AngleAxisd(turn_radians_per_step, Eigen::Vector3d::UnitZ()));
  Eigen::Vector3d local_offset(0., nominal_footwidth / 2., 0);
  for (int i(0); i < n_steps; ++i) {
    rotated_mid_foot.SetOri(quat_increment_local *
                            rotated_mid_foot.GetOrientation());

    new_left_foot.SetPosOriSide(
        rotated_mid_foot.GetPos() + rotated_mid_foot.GetRotMat() * local_offset,
        rotated_mid_foot.GetOrientation(), end_effector::LFoot);
    new_right_foot.SetPosOriSide(
        rotated_mid_foot.GetPos() +
            rotated_mid_foot.GetRotMat() * -local_offset,
        rotated_mid_foot.GetOrientation(), end_effector::RFoot);
    if (turn_radians_per_step > 0) {
      foot_step_list.push_back(new_left_foot);
      foot_step_list.push_back(new_right_foot);
    } else {
      foot_step_list.push_back(new_right_foot);
      foot_step_list.push_back(new_left_foot);
    }
  }
  return foot_step_list;
}

std::vector<FootStep>
FootStep::GetStrafeFootStep(const int n_steps, const double strafe_distance,
                            const double nominal_footwidth,
                            const FootStep &current_mid_foot) {
  std::vector<FootStep> foot_step_list;

  FootStep new_right_foot, new_left_foot;
  FootStep strafed_mid_foot = current_mid_foot;

  Eigen::Vector3d strafe_vec(0., strafe_distance, 0.);
  Eigen::Vector3d local_offset(0., nominal_footwidth / 2., 0.);
  for (int i(0); i < n_steps; ++i) {
    strafed_mid_foot.SetPos(strafed_mid_foot.GetPos() +
                            strafed_mid_foot.GetRotMat() * strafe_vec);

    new_left_foot.SetPosOriSide(
        strafed_mid_foot.GetPos() + strafed_mid_foot.GetRotMat() * local_offset,
        strafed_mid_foot.GetOrientation(), end_effector::LFoot);
    new_right_foot.SetPosOriSide(
        strafed_mid_foot.GetPos() +
            strafed_mid_foot.GetRotMat() * -local_offset,
        strafed_mid_foot.GetOrientation(), end_effector::RFoot);

    if (strafe_distance > 0) {
      foot_step_list.push_back(new_left_foot);
      foot_step_list.push_back(new_right_foot);
    } else {
      foot_step_list.push_back(new_right_foot);
      foot_step_list.push_back(new_left_foot);
    }
  }
  return foot_step_list;
}

void FootStep::ComputeMidFoot(const FootStep &footstep1,
                              const FootStep &footstep2, FootStep &midfoot) {
  Eigen::Vector3d mid_pos = 0.5 * (footstep1.GetPos() + footstep2.GetPos());
  Eigen::Quaterniond mid_quat =
      footstep1.GetOrientation().slerp(0.5, footstep2.GetOrientation());
  midfoot.SetPosOri(mid_pos, mid_quat);
  midfoot.SetMidFoot();
}

void FootStep::MakeHorizontal(Eigen::Isometry3d &pose) {
  const Eigen::Matrix3d R = pose.linear();
  const Eigen::Vector3d p = pose.translation();
  Eigen::Vector3d rpy = util::RPYFromSO3(R);
  pose.translation() = Eigen::Vector3d{p(0), p(1), 0.};
  pose.linear() = util::SO3FromRPY(0., 0., rpy(2));
}

Eigen::Isometry3d FootStep::MakeIsometry(const FootStep &foot_step) {
  Eigen::Isometry3d ret;
  ret.translation() = foot_step.GetPos();
  ret.linear() = foot_step.GetRotMat();
  return ret;
}

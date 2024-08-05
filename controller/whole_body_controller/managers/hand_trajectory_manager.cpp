#include "controller/whole_body_controller/managers/hand_trajectory_manager.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "util/interpolation.hpp"
#include "util/util.hpp"

HandTrajectoryManager::HandTrajectoryManager(Task *pos_task, Task *ori_task,
                                             PinocchioRobotSystem *robot)
    : pos_task_(pos_task), ori_task_(ori_task), robot_(robot) {
  util::PrettyConstructor(2, "HandTrajectoryManager");
  target_pos_ = Eigen::VectorXd::Zero(3);
  target_ori_ = Eigen::Quaterniond::Identity();
  delta_pos_speed_ = .5;

  pos_curve_ = new HermiteCurveVec();
  ori_curve_ = new HermiteQuaternionCurve();
}

HandTrajectoryManager::~HandTrajectoryManager() {
  delete pos_curve_;
  delete ori_curve_;
}

void HandTrajectoryManager::InitializeHandTrajectory(
    const Eigen::Isometry3d &target_pose, const double start_time,
    const double duration, const bool initialized) {
  start_time_ = start_time;
  duration_ = duration;

  double delta_pos_norm;
  Eigen::Quaterniond init_ori;
  Eigen::VectorXd init_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd init_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd end_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd cmd_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd delta_pos = Eigen::VectorXd::Zero(3);

  if (!initialized) {
    init_pos = robot_->GetLinkIsometry(pos_task_->TargetIdx()).translation();
    init_ori = robot_->GetLinkIsometry(ori_task_->TargetIdx()).linear();
    // std::cout << "Initialize Hand Trajectory" << std::endl;
  } else {
    init_pos = target_pos_;
    init_ori = target_ori_;
    // std::cout << "Update Hand Trajectory" << std::endl;
  }

  cmd_pos << target_pose.translation();
  delta_pos = cmd_pos - init_pos;
  delta_pos_norm = delta_pos.norm();

  // manual filtering if target pos error is too large
  if (delta_pos_norm > duration_ * delta_pos_speed_) {
    target_pos_ =
        (duration_ * delta_pos_speed_ / delta_pos_norm) * delta_pos + init_pos;
  } else {
    target_pos_ = cmd_pos;
  }

  target_ori_ = Eigen::Quaterniond(target_pose.linear());

  pos_curve_->Initialize(init_pos, init_vel, target_pos_, end_vel, duration_);
  ori_curve_->Initialize(init_ori, init_vel, target_ori_, end_vel, duration_);
}

void HandTrajectoryManager::UpdateHandPose(const double current_time) {
  double t = current_time - start_time_;
  Eigen::VectorXd des_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd des_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd des_acc = Eigen::VectorXd::Zero(3);
  Eigen::Quaterniond des_ori_quat;
  Eigen::VectorXd des_ori = Eigen::VectorXd::Zero(4);
  Eigen::Vector3d des_ang_vel = Eigen::Vector3d::Zero();
  Eigen::VectorXd des_ang_acc = Eigen::VectorXd::Zero(3);

  des_pos << pos_curve_->Evaluate(t);
  ori_curve_->Evaluate(t, des_ori_quat);
  des_ori << des_ori_quat.normalized().coeffs();

  des_vel << pos_curve_->EvaluateFirstDerivative(t);
  ori_curve_->GetAngularVelocity(t, des_ang_vel);

  pos_task_->UpdateDesired(des_pos, des_vel, des_ang_acc);
  ori_task_->UpdateDesired(des_ori, des_ang_vel, des_ang_acc);
}

void HandTrajectoryManager::UpdateDesired(
    const Eigen::Isometry3d &target_pose) {
  Eigen::VectorXd target_pos = Eigen::VectorXd::Zero(3);
  target_pos << target_pose.translation();

  Eigen::Quaterniond target_ori_quat(target_pose.linear());

  Eigen::VectorXd target_ori(4);
  target_ori << target_ori_quat.normalized().coeffs();

  pos_task_->UpdateDesired(target_pos, Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero());
  ori_task_->UpdateDesired(target_ori, Eigen::Vector3d::Zero(),
                           Eigen::Vector3d::Zero());
}

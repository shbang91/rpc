#include "controller/whole_body_controller/managers/hand_trajectory_manager.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "util/interpolation.hpp"
#include "util/util.hpp"

HandTrajectoryManager::HandTrajectoryManager(Task *pos_task, Task *ori_task,
                                             PinocchioRobotSystem *robot)
    : pos_task_(pos_task), ori_task_(ori_task), robot_(robot),
      pos_curve_(nullptr), ori_curve_(nullptr) {
  util::PrettyConstructor(2, "HandTrajectoryManager");
  target_pos_ = Eigen::Vector3d::Zero();
  target_ori_ = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
  delta_pos_speed_ = 0.1;
}

HandTrajectoryManager::~HandTrajectoryManager() {
  if (pos_curve_ != nullptr)
    delete pos_curve_;
  if (ori_curve_ != nullptr)
    delete ori_curve_;
}

void HandTrajectoryManager::UseCurrent() {
  Eigen::VectorXd des_pos(3);
  des_pos << robot_->GetLinkIsometry(pos_task_->TargetIdx()).translation();
  Eigen::VectorXd des_vel(3);
  des_vel << robot_->GetLinkSpatialVel(pos_task_->TargetIdx()).tail<3>();
  Eigen::VectorXd des_acc = Eigen::VectorXd::Zero(3);

  Eigen::Quaterniond des_ori_quat(
      robot_->GetLinkIsometry(ori_task_->TargetIdx()).linear());
  Eigen::VectorXd des_ori(4);
  des_ori << des_ori_quat.normalized().coeffs();
  Eigen::VectorXd des_ang_vel(3);
  des_ang_vel << robot_->GetLinkSpatialVel(ori_task_->TargetIdx()).head<3>();

  pos_task_->UpdateDesired(des_pos, des_vel, des_acc);
  ori_task_->UpdateDesired(des_ori, des_ang_vel, des_acc);
}

void HandTrajectoryManager::InitializeHandTrajectory(
    const Eigen::Isometry3d &target_pose, const double start_time,
    const double duration, const int initialized) {
  start_time_ = start_time;
  duration_ = duration;

  double delta_pos_norm;
  Eigen::Quaterniond init_ori;
  Eigen::VectorXd init_vel = Eigen::VectorXd::Zero(3);
  ;
  Eigen::VectorXd cmd_pos(3);
  Eigen::VectorXd delta_pos(3);
  Eigen::VectorXd init_pos(target_pos_);

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

  if (delta_pos_norm > duration_ * delta_pos_speed_) {
    target_pos_ =
        (duration_ * delta_pos_speed_ / delta_pos_norm) * delta_pos + init_pos;
  } else {
    target_pos_ = cmd_pos;
  }

  target_ori_ = target_pose.linear();

  pos_curve_ = new HermiteCurveVec(init_pos, init_vel, target_pos_,
                                   Eigen::Vector3d::Zero(), duration_);
  ori_curve_ = new HermiteQuaternionCurve(init_ori, init_vel, target_ori_,
                                          Eigen::Vector3d::Zero(), duration_);
}

void HandTrajectoryManager::UpdateHandPose(const double current_time) {
  double s = (current_time - start_time_) / duration_;
  Eigen::VectorXd des_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd des_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd des_acc = Eigen::VectorXd::Zero(3);
  Eigen::Quaterniond des_ori_quat;
  Eigen::VectorXd des_ori = Eigen::VectorXd::Zero(4);
  Eigen::VectorXd des_ang_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd des_ang_acc = Eigen::VectorXd::Zero(3);

  des_pos << pos_curve_->Evaluate(current_time - start_time_);
  ori_curve_->Evaluate(current_time - start_time_, des_ori_quat);
  des_ori << des_ori_quat.normalized().coeffs();

  // des_vel << pos_curve_->EvaluateFirstDerivative(current_time - start_time_);
  // ori_curve_->GetAngularVelocity(current_time - start_time_, des_ang_vel);

  if (pos_task_ != nullptr)
    pos_task_->UpdateDesired(des_pos, des_vel, des_ang_acc);
  if (ori_task_ != nullptr)
    ori_task_->UpdateDesired(des_ori, des_ang_vel, des_ang_acc);
}

void HandTrajectoryManager::UpdateDesired(
    const Eigen::Isometry3d &target_pose) {
  Eigen::VectorXd target_pos = Eigen::VectorXd::Zero(3);
  target_pos << target_pose.translation();

  Eigen::Quaterniond target_ori_quat(target_pose.linear());

  Eigen::VectorXd target_ori(4);
  target_ori << target_ori_quat.normalized().coeffs();

  if (pos_task_ != nullptr)
    pos_task_->UpdateDesired(target_pos, Eigen::Vector3d::Zero(),
                             Eigen::Vector3d::Zero());
  if (ori_task_ != nullptr)
    ori_task_->UpdateDesired(target_ori, Eigen::Vector3d::Zero(),
                             Eigen::Vector3d::Zero());
}

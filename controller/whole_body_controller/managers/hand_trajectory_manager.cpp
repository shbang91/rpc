#include "controller/whole_body_controller/managers/hand_trajectory_manager.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "util/interpolation.hpp"
#include "util/util.hpp"

HandTrajectoryManager::HandTrajectoryManager
(Task *pos_task, Task *ori_task, PinocchioRobotSystem *robot)
  : pos_task_(pos_task), ori_task_(ori_task), robot_(robot) 
{
  util::PrettyConstructor(2, "HandTrajectoryManager");
}

HandTrajectoryManager::~HandTrajectoryManager() 
{
  if (ori_curve_ != nullptr)
    delete ori_curve_;
}

void HandTrajectoryManager::UseCurrent() 
{
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

void HandTrajectoryManager::InitializeHandTrajectory(const Eigen::Isometry3d &target_pose, const double start_time, const double duration) 
{
  start_time_ = start_time;
  duration_ = duration;

  Eigen::VectorXd init_pos(3);
  init_pos_ << robot_->GetLinkIsometry(pos_task_->TargetIdx()).translation();
  Eigen::VectorXd init_vel(3);
  init_vel << robot_->GetLinkSpatialVel(pos_task_->TargetIdx()).tail<3>();

  Eigen::Quaterniond init_ori(
      robot_->GetLinkIsometry(ori_task_->TargetIdx()).linear());

  Eigen::VectorXd target_pos = Eigen::VectorXd::Zero(3);
  target_pos_ << target_pose.translation();
  Eigen::Quaterniond target_ori(target_pose.linear());

  Eigen::VectorXd target_vel = Eigen::VectorXd::Zero(3);

  ori_curve_ = 
    new HermiteQuaternionCurve(init_ori, init_vel, target_ori, Eigen::Vector3d::Zero(), duration_);
}

void HandTrajectoryManager::UpdateHandPose(const double current_time)
{
  double s = (current_time - start_time_) / duration_;
  Eigen::VectorXd target_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd target_vel = Eigen::VectorXd::Zero(3);
  Eigen::Quaterniond target_ori_quat;
  Eigen::VectorXd target_ori = Eigen::VectorXd::Zero(3);

  for (int i(0); i<3; ++i) 
  {
    target_pos[i] = util::SmoothPos(
        init_pos_[i], target_pos_[i], duration_, current_time - start_time_);
    target_vel[i] = util::SmoothVel(
        init_pos_[i], target_pos_[i], duration_, current_time - start_time_);
  }

  ori_curve_->Evaluate(current_time - start_time_, target_ori_quat);
  target_ori << target_ori_quat.normalized().coeffs();

  pos_task_->UpdateDesired(target_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  if (ori_task_ != nullptr)
    ori_task_->UpdateDesired(target_ori, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());    
}

void HandTrajectoryManager::UpdateDesired(const Eigen::Isometry3d &target_pose)
{
  Eigen::VectorXd target_pos = Eigen::VectorXd::Zero(3);
  target_pos << target_pose.translation();
  Eigen::Quaterniond target_ori_quat(target_pose.linear());
  Eigen::VectorXd target_ori(4);
  target_ori << target_ori_quat.normalized().coeffs();

  pos_task_->UpdateDesired(target_pos, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  if (ori_task_ != nullptr)
    ori_task_->UpdateDesired(target_ori, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());    
}
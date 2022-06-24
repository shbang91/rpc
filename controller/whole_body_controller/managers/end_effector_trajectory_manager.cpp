#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"

EndEffectorTrajectoryManager::EndEffectorTrajectoryManager(
    Task *pos_task, Task *ori_task, PinocchioRobotSystem *robot)
    : pos_task_(pos_task), ori_task_(ori_task), robot_(robot) {
  util::PrettyConstructor(2, "EndEffectorTrajectoryManager");
}

void EndEffectorTrajectoryManager::UseCurrent() {
  // pos desired
  // Eigen::VectorXd des_pos = Eigen::VectorXd::Zero(3);
  // Eigen::VectorXd des_vel = Eigen::VectorXd::Zero(3);

  // Eigen::VectorXd des_ori = Eigen::VectorXd::Zero(4);
  // Eigen::VectorXd des_ang_vel = Eigen::VectorXd::Zero(3);

  // TODO: check
  Eigen::VectorXd des_pos =
      robot_->GetLinkIsometry(pos_task_->TargetIdx()).translation();
  Eigen::VectorXd des_vel =
      robot_->GetLinkSpatialVel(pos_task_->TargetIdx()).tail<3>();
  Eigen::Quaterniond des_ori_quat(
      robot_->GetLinkIsometry(ori_task_->TargetIdx()).linear());
  Eigen::VectorXd des_ang_vel =
      robot_->GetLinkSpatialVel(ori_task_->TargetIdx()).head<3>();

  Eigen::VectorXd des_ori(des_ori_quat.coeffs());

  pos_task_->UpdateDesired(des_pos, des_vel, Eigen::VectorXd::Zero(3));
  ori_task_->UpdateDesired(des_ori, des_ang_vel, Eigen::VectorXd::Zero(3));
}

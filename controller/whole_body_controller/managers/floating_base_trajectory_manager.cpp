#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "util/interpolation.hpp"
#include "util/util.hpp"

FloatingBaseTrajectoryManager::FloatingBaseTrajectoryManager(
    Task *com_task, Task *torso_ori_task, PinocchioRobotSystem *robot)
    : com_task_(com_task), torso_ori_task_(torso_ori_task), robot_(robot),
      duration_(0.), init_com_pos_(Eigen::Vector3d::Zero()),
      target_com_pos_(Eigen::Vector3d::Zero()),
      exp_err_(Eigen::VectorXd::Zero(3)) {
  util::PrettyConstructor(2, "FloatingBaseTrajectoryManager");
}

void FloatingBaseTrajectoryManager::InitializeFloatingBaseInterpolation(
    const Eigen::Vector3d &target_com_pos,
    const Eigen::Quaterniond &target_torso_quat, const double duration,
    const bool b_use_base_height) {
  duration_ = duration;
  init_com_pos_ = robot_->GetRobotComPos();
  target_com_pos_ = target_com_pos;
  target_com_pos_[2] =
      b_use_base_height
          ? robot_->GetLinkIsometry(torso_ori_task_->GetTargetIdx())
                .translation()[2]
          : target_com_pos_[2];

  init_torso_quat_ = Eigen::Quaterniond(
      robot_->GetLinkIsometry(torso_ori_task_->GetTargetIdx()).linear());
  exp_err_ = util::QuatToExp(target_torso_quat * init_torso_quat_.inverse());
}

void FloatingBaseTrajectoryManager::UpdateDesired(
    const double state_machine_time) {
  Eigen::VectorXd des_com_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd des_com_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd des_com_acc = Eigen::VectorXd::Zero(3);

  for (int i(0); i < des_com_pos.size(); ++i) {
    des_com_pos[i] = util::SmoothPos(init_com_pos_[i], target_com_pos_[i],
                                     duration_, state_machine_time);
    des_com_vel[i] = util::SmoothVel(init_com_pos_[i], target_com_pos_[i],
                                     duration_, state_machine_time);
    des_com_acc[i] = util::SmoothAcc(init_com_pos_[i], target_com_pos_[i],
                                     duration_, state_machine_time);
  }
  double t = util::SmoothPos(0, 1, duration_, state_machine_time);
  double t_dot = util::SmoothVel(0, 1, duration_, state_machine_time);
  double t_ddot = util::SmoothAcc(0, 1, duration_, state_machine_time);

  Eigen::Quaterniond des_torso_quat =
      util::ExpToQuat(exp_err_ * t) * init_torso_quat_;
  Eigen::VectorXd des_torso_quat_vec(des_torso_quat.normalized().coeffs());

  Eigen::VectorXd des_torso_ang_vel(exp_err_ * t_dot);
  Eigen::VectorXd des_torso_ang_acc(exp_err_ * t_ddot);

  com_task_->UpdateDesiredTask(des_com_pos, des_com_vel, des_com_acc);
  torso_ori_task_->UpdateDesiredTask(des_torso_quat_vec, des_torso_ang_vel,
                                     des_torso_ang_acc);
}

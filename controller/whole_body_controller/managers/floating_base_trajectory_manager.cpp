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
      exp_err_(Eigen::VectorXd::Zero(3)), amp_(Eigen::Vector3d::Zero()),
      freq_(Eigen::Vector3d::Zero()), b_swaying_(false) {
  util::PrettyConstructor(2, "FloatingBaseTrajectoryManager");
}

void FloatingBaseTrajectoryManager::InitializeFloatingBaseInterpolation(
    const Eigen::Vector3d &init_com_pos, const Eigen::Vector3d &target_com_pos,
    const Eigen::Quaterniond &init_torso_quat,
    const Eigen::Quaterniond &target_torso_quat, const double duration) {
  duration_ = duration;
  init_com_pos_ = init_com_pos;
  target_com_pos_ = target_com_pos;

  init_torso_quat_ = init_torso_quat;
  exp_err_ = util::QuatToExp(target_torso_quat * init_torso_quat_.inverse());
}

void FloatingBaseTrajectoryManager::UpdateDesired(
    const double state_machine_time) {
  Eigen::VectorXd des_com_pos = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd des_com_vel = Eigen::VectorXd::Zero(3);
  Eigen::VectorXd des_com_acc = Eigen::VectorXd::Zero(3);

  if (b_swaying_) {
    // com swaying
    util::SinusoidTrajectory(init_com_pos_, amp_, freq_, state_machine_time,
                             des_com_pos, des_com_vel, des_com_acc, 1.0);

    // update desired com task
    com_task_->UpdateDesired(des_com_pos, des_com_vel, des_com_acc);

  } else {
    // com & torso ori smooth interpolation TODO: change to minjerk traj
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
    Eigen::VectorXd des_torso_quat_vec(4);
    des_torso_quat_vec << des_torso_quat.normalized().coeffs();

    Eigen::VectorXd des_torso_ang_vel(3);
    des_torso_ang_vel << exp_err_ * t_dot;
    Eigen::VectorXd des_torso_ang_acc(3);
    des_torso_ang_acc << exp_err_ * t_ddot;

    // update desired com & torso_ori task
    com_task_->UpdateDesired(des_com_pos, des_com_vel, des_com_acc);
    torso_ori_task_->UpdateDesired(des_torso_quat_vec, des_torso_ang_vel,
                                   des_torso_ang_acc);
  }
}

void FloatingBaseTrajectoryManager::InitializeSwaying(
    const Eigen::Vector3d &init_com_pos, const Eigen::Vector3d &amp,
    const Eigen::Vector3d &freq) {
  init_com_pos_ = init_com_pos;
  amp_ = amp;
  freq_ = freq;
  b_swaying_ = true;
}

#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "util/interpolation.hpp"
#include "util/util.hpp"

FloatingBaseTrajectoryManager::FloatingBaseTrajectoryManager(
    Task *com_xy_task, Task *com_z_task, Task *torso_ori_task,
    PinocchioRobotSystem *robot)
    : com_xy_task_(com_xy_task), com_z_task_(com_z_task),
      torso_ori_task_(torso_ori_task), robot_(robot), duration_(0.),
      init_com_pos_(Eigen::Vector3d::Zero()),
      target_com_pos_(Eigen::Vector3d::Zero()),
      exp_err_(Eigen::VectorXd::Zero(3)), amp_(Eigen::Vector3d::Zero()),
      freq_(Eigen::Vector3d::Zero()), b_swaying_(false),
      min_jerk_curve_(nullptr), min_jerk_time_(nullptr) {

  util::PrettyConstructor(2, "FloatingBaseTrajectoryManager");
}

FloatingBaseTrajectoryManager::~FloatingBaseTrajectoryManager() {
  if (min_jerk_curve_ != nullptr)
    delete min_jerk_curve_;
  if (min_jerk_time_ != nullptr)
    delete min_jerk_time_;
}

void FloatingBaseTrajectoryManager::InitializeFloatingBaseInterpolation(
    const Eigen::Vector3d &init_com_pos, const Eigen::Vector3d &target_com_pos,
    const Eigen::Quaterniond &init_torso_quat,
    const Eigen::Quaterniond &target_torso_quat, const double duration) {

  duration_ = duration;

  // linear
  init_com_pos_ = init_com_pos;
  target_com_pos_ = target_com_pos;
  Eigen::VectorXd start_pos(3), end_pos(3);
  start_pos << init_com_pos_[0], init_com_pos_[1], init_com_pos_[2];
  end_pos << target_com_pos_[0], target_com_pos_[1], target_com_pos[2];
  min_jerk_curve_ = new MinJerkCurveVec(
      start_pos, Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3), end_pos,
      Eigen::VectorXd::Zero(3), Eigen::VectorXd::Zero(3), duration_);

  // angular
  init_torso_quat_ = init_torso_quat;
  exp_err_ = util::QuatToExp(target_torso_quat * init_torso_quat_.inverse());
  Eigen::VectorXd start_time(1), end_time(1);
  start_time << 0.;
  end_time << 1.;
  min_jerk_time_ = new MinJerkCurveVec(
      start_time, Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), end_time,
      Eigen::VectorXd::Zero(1), Eigen::VectorXd::Zero(1), duration_);
}

void FloatingBaseTrajectoryManager::InitializeSwaying(
    const Eigen::Vector3d &init_com_pos, const Eigen::Vector3d &amp,
    const Eigen::Vector3d &freq, const Eigen::Matrix3d &rot_world_local) {
  init_com_pos_ = init_com_pos;
  amp_ = amp;
  freq_ = freq;
  b_swaying_ = true;
  rot_world_local_ = rot_world_local;
}

void FloatingBaseTrajectoryManager::UpdateDesired(
    const double state_machine_time) {
  if (b_swaying_) {

    Eigen::VectorXd local_des_pos = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd local_des_vel = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd local_des_acc = Eigen::VectorXd::Zero(3);

    // com swaying
    util::SinusoidTrajectory(amp_, freq_, state_machine_time, local_des_pos,
                             local_des_vel, local_des_acc, 1.0);

    Eigen::VectorXd des_com_pos =
        init_com_pos_ + rot_world_local_ * local_des_pos;
    Eigen::VectorXd des_com_vel = rot_world_local_ * local_des_vel;
    Eigen::VectorXd des_com_acc = rot_world_local_ * local_des_acc;
    // update desired com task
    com_xy_task_->UpdateDesired(des_com_pos.head<2>(), des_com_vel.head<2>(),
                                des_com_acc.head<2>());
    com_z_task_->UpdateDesired(des_com_pos.tail<1>(), des_com_vel.tail<1>(),
                               des_com_acc.tail<1>());

  } else {
    // minjerk com traj generation
    if (min_jerk_curve_ == nullptr || min_jerk_time_ == nullptr)
      throw std::runtime_error(
          "Initialze MinJerkCurve First in FlaotingBaseTrajectoryManager");

    Eigen::VectorXd des_com_pos = min_jerk_curve_->Evaluate(state_machine_time);
    Eigen::VectorXd des_com_vel =
        min_jerk_curve_->EvaluateFirstDerivative(state_machine_time);
    Eigen::VectorXd des_com_acc =
        min_jerk_curve_->EvaluateSecondDerivative(state_machine_time);

    // update com des traj
    com_xy_task_->UpdateDesired(des_com_pos.head<2>(), des_com_vel.head<2>(),
                                des_com_acc.head<2>());
    com_z_task_->UpdateDesired(des_com_pos.tail<1>(), des_com_vel.tail<1>(),
                               des_com_acc.tail<1>());

    // torso ori traj generation
    double t = min_jerk_time_->Evaluate(state_machine_time)[0];
    double t_dot =
        min_jerk_time_->EvaluateFirstDerivative(state_machine_time)[0];
    double t_ddot =
        min_jerk_time_->EvaluateSecondDerivative(state_machine_time)[0];

    Eigen::Quaterniond des_torso_quat =
        util::ExpToQuat(exp_err_ * t) * init_torso_quat_;
    Eigen::VectorXd des_torso_quat_vec(4);
    des_torso_quat_vec << des_torso_quat.normalized().coeffs();

    Eigen::VectorXd des_torso_ang_vel(3);
    des_torso_ang_vel << exp_err_ * t_dot;
    Eigen::VectorXd des_torso_ang_acc(3);
    des_torso_ang_acc << exp_err_ * t_ddot;

    // update desired torso_ori des traj
    torso_ori_task_->UpdateDesired(des_torso_quat_vec, des_torso_ang_vel,
                                   des_torso_ang_acc);
  }
}

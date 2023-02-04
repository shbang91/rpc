#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "util/interpolation.hpp"
#include "util/util.hpp"

EndEffectorTrajectoryManager::EndEffectorTrajectoryManager(
    Task *pos_task, Task *ori_task, PinocchioRobotSystem *robot)
    : pos_task_(pos_task), ori_task_(ori_task), robot_(robot),
      pos_first_half_curve_(nullptr), pos_second_half_curve_(nullptr),
      ori_curve_(nullptr) {
  util::PrettyConstructor(2, "EndEffectorTrajectoryManager");
}

EndEffectorTrajectoryManager::~EndEffectorTrajectoryManager() {
  if (pos_first_half_curve_ != nullptr)
    delete pos_first_half_curve_;
  if (pos_second_half_curve_ != nullptr)
    delete pos_second_half_curve_;
  if (ori_curve_ != nullptr)
    delete ori_curve_;
}

void EndEffectorTrajectoryManager::UseCurrent() {

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

void EndEffectorTrajectoryManager::InitializeSwingTrajectory(
    const Eigen::Isometry3d &ini_pose, const Eigen::Isometry3d &fin_pose,
    const double swing_height, const double duration) {

  duration_ = duration;

  Eigen::VectorXd start_pos = Eigen::VectorXd::Zero(3);
  start_pos << ini_pose.translation();

  Eigen::VectorXd end_pos = Eigen::VectorXd::Zero(3);
  end_pos << fin_pose.translation();

  Eigen::VectorXd mid_swing_pos = (start_pos + end_pos) / 2.;
  mid_swing_pos[2] = swing_height;
  Eigen::VectorXd avg_swing_vel = (end_pos - start_pos) / duration_;

  // hermite curve
  pos_first_half_curve_ =
      new HermiteCurveVec(start_pos, Eigen::VectorXd::Zero(3), mid_swing_pos,
                          avg_swing_vel, 0.5 * duration_);
  pos_second_half_curve_ =
      new HermiteCurveVec(mid_swing_pos, avg_swing_vel, end_pos,
                          Eigen::VectorXd::Zero(3), 0.5 * duration_);

  Eigen::Quaterniond start_ori(ini_pose.linear());
  Eigen::Quaterniond end_ori(fin_pose.linear());

  ori_curve_ =
      new HermiteQuaternionCurve(start_ori, Eigen::Vector3d::Zero(), end_ori,
                                 Eigen::Vector3d::Zero(), duration_);
}

void EndEffectorTrajectoryManager::UpdateDesired(const double current_time) {
  assert(pos_first_half_curve_ != nullptr);
  assert(pos_second_half_curve_ != nullptr);
  assert(ori_curve_ != nullptr);

  Eigen::VectorXd des_pos =
      current_time < 0.5 * duration_
          ? pos_first_half_curve_->Evaluate(current_time)
          : pos_second_half_curve_->Evaluate(current_time - 0.5 * duration_);

  Eigen::VectorXd des_vel =
      current_time < 0.5 * duration_
          ? pos_first_half_curve_->EvaluateFirstDerivative(current_time)
          : pos_second_half_curve_->EvaluateFirstDerivative(current_time -
                                                            0.5 * duration_);

  Eigen::VectorXd des_acc =
      current_time < 0.5 * duration_
          ? pos_first_half_curve_->EvaluateSecondDerivative(current_time)
          : pos_second_half_curve_->EvaluateSecondDerivative(current_time -
                                                             0.5 * duration_);

  pos_task_->UpdateDesired(des_pos, des_vel, des_acc);

  Eigen::Quaterniond des_quat = Eigen::Quaterniond::Identity();
  ori_curve_->Evaluate(current_time, des_quat);

  Eigen::Vector3d des_ori_vel_vec = Eigen::Vector3d::Zero();
  ori_curve_->GetAngularVelocity(current_time, des_ori_vel_vec);

  Eigen::Vector3d des_ori_acc_vec = Eigen::Vector3d::Zero();
  ori_curve_->GetAngularAcceleration(current_time, des_ori_acc_vec);

  Eigen::VectorXd des_ori(4);
  des_ori << des_quat.coeffs();
  Eigen::VectorXd des_ori_vel(3);
  des_ori_vel << des_ori_vel_vec;
  Eigen::VectorXd des_ori_acc(3);
  des_ori_acc << des_ori_acc_vec;

  ori_task_->UpdateDesired(des_ori, des_ori_vel, des_ori_acc);
}

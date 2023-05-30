#include "controller/whole_body_controller/managers/reaction_force_trajectory_manager.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "util/util.hpp"
#include "util/interpolation.hpp"

ForceTrajectoryManager::ForceTrajectoryManager(ForceTask *force_task,
                                               PinocchioRobotSystem *robot)
    : force_task_(force_task), robot_(robot), des_init_rf_(force_task_->Dim()),
      des_final_rf_(force_task_->Dim()), grav_vec_(force_task_->Dim()), duration_(0.),
      b_swaying_(false), amp_(Eigen::Vector3d::Zero()), freq_(Eigen::Vector3d::Zero()){

  grav_vec_ << 0., 0., 0., 0., 0., 9.81;

  util::PrettyConstructor(2, "ReactionForceTrajectoryManager");
}

void ForceTrajectoryManager::InitializeInterpolation(
    const Eigen::VectorXd &des_init, const Eigen::VectorXd &des_fin,
    double duration) {

  assert(des_init.size() == des_init_rf_.size());
  assert(des_fin.size() == des_final_rf_.size());

  des_init_rf_ = des_init;
  des_final_rf_ = des_fin;
  duration_ = duration;
}

void ForceTrajectoryManager::InitializeInterpolation(
    const Eigen::VectorXd &des_fin,
    double duration) {

  assert(des_fin.size() == des_final_rf_.size());

  InitializeInterpolation(des_final_rf_, des_fin, duration);
}

void ForceTrajectoryManager::InitializeSwaying(
        const Eigen::VectorXd &init_des_force,  const Eigen::Vector3d &amp,
        const Eigen::Vector3d &freq) {

  assert(init_des_force.size() == des_init_rf_.size());

  des_init_rf_ = init_des_force;
  amp_ = amp;
  freq_ = freq;
  b_swaying_ = true;
}

void ForceTrajectoryManager::UpdateDesired(double query_time) {
  if (b_swaying_) {
    Eigen::VectorXd des_com_pos = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd des_com_vel = Eigen::VectorXd::Zero(3);
    Eigen::VectorXd des_com_acc = Eigen::VectorXd::Zero(3);

    // compute desired CoM pos, vel, and accel
    util::SinusoidTrajectory(des_init_rf_, amp_, freq_, query_time,
                             des_com_pos, des_com_vel, des_com_acc, 1.0);

    Eigen::VectorXd des_rf = Eigen::VectorXd::Zero(6);
    Eigen::VectorXd aug_ext_des_com_acc = Eigen::VectorXd::Zero(6);
    aug_ext_des_com_acc.tail(3) = des_com_acc;
    des_rf = robot_->GetTotalMass()/2.0 * (aug_ext_des_com_acc + grav_vec_);
    _ConvertToLocalDesired(des_rf);
    force_task_->UpdateDesired(des_rf);
  } else {
    query_time = util::Clamp(query_time, 0., duration_);

    Eigen::VectorXd des_rf =
            des_init_rf_ + (des_final_rf_ - des_init_rf_) / duration_ * query_time;

    _ConvertToLocalDesired(des_rf);

    force_task_->UpdateDesired(des_rf);
  }

}

Eigen::VectorXd ForceTrajectoryManager::GetFinalDesiredRf() {
  return des_final_rf_;
}

void ForceTrajectoryManager::_ConvertToLocalDesired(Eigen::VectorXd &des_rf) {
  Eigen::MatrixXd rot_l_w = Eigen::MatrixXd::Zero(des_rf.size(), des_rf.size());
  rot_l_w.block<3, 3>(0, 0) =
      robot_->GetLinkIsometry(force_task_->contact()->TargetLinkIdx())
          .linear()
          .transpose();

  // consider surface contact
  if (des_rf.size() == 6)
    rot_l_w.block<3, 3>(3, 3) =
        robot_->GetLinkIsometry(force_task_->contact()->TargetLinkIdx())
            .linear()
            .transpose();

  des_rf = rot_l_w * des_rf;
}

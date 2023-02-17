#include "controller/whole_body_controller/managers/reaction_force_trajectory_manager.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "util/util.hpp"

ForceTrajectoryManager::ForceTrajectoryManager(ForceTask *force_task,
                                               PinocchioRobotSystem *robot)
    : force_task_(force_task), robot_(robot), des_init_rf_(force_task_->Dim()),
      des_final_rf_(force_task_->Dim()), duration_(0.) {

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

void ForceTrajectoryManager::UpdateDesired(double query_time) {
  query_time = util::Clamp(query_time, 0., duration_);

  Eigen::VectorXd des_rf =
      des_init_rf_ + (des_final_rf_ - des_init_rf_) / duration_ * query_time;

  _ConvertToLocalDesired(des_rf);

  force_task_->UpdateDesired(des_rf);
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

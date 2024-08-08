#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"

TaskHierarchyManager::TaskHierarchyManager(Task *task, Eigen::VectorXd w_max,
                                           Eigen::VectorXd w_min)
    : task_(task), w_max_(w_max), w_min_(w_min),
      w_start_(Eigen::VectorXd ::Zero(task->Dim())),
      w_fin_(Eigen::VectorXd::Zero(task->Dim())), duration_(0.) {
  util::PrettyConstructor(2, "TaskHierarchyManager");
}

void TaskHierarchyManager::InitializeRampToMax(const double interp_duration) {
  w_start_ = task_->Weight();
  w_fin_ = w_max_;
  duration_ = interp_duration;
}

void TaskHierarchyManager::InitializeRampToMin(const double interp_duration) {
  w_start_ = task_->Weight();
  w_fin_ = w_min_;
  duration_ = interp_duration;
}

void TaskHierarchyManager::UpdateRampToMax(const double state_machine_time) {
  Eigen::VectorXd w_current =
      w_start_ + (w_fin_ - w_start_) / duration_ * state_machine_time;

  if (state_machine_time >= duration_)
    w_current = w_fin_;

  task_->SetWeight(w_current);
}

void TaskHierarchyManager::UpdateRampToMin(const double state_machine_time) {
  Eigen::VectorXd w_current =
      w_start_ + (w_fin_ - w_start_) / duration_ * state_machine_time;

  if (state_machine_time >= duration_)
    w_current = w_fin_;

  task_->SetWeight(w_current);
}

void TaskHierarchyManager::SetWeightMax(const Eigen::Vector3d &w_max) {
  w_max_ = w_max;
}

void TaskHierarchyManager::SetWeightMin(const Eigen::Vector3d &w_min) {
  w_min_ = w_min;
}

void TaskHierarchyManager::SetTaskKp(const Eigen::Vector3d &kp) {
  task_->SetKp(kp);
}

void TaskHierarchyManager::SetTaskKd(const Eigen::Vector3d &kd) {
  task_->SetKd(kd);
}

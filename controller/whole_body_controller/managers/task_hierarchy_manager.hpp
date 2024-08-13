#pragma once
#include <Eigen/Dense>
#include "controller/whole_body_controller/task.hpp"
class Task;

class TaskHierarchyManager {
public:
  TaskHierarchyManager(Task *task, Eigen::VectorXd w_max,
                       Eigen::VectorXd w_min);
  virtual ~TaskHierarchyManager() = default;

  void InitializeRampToMax(const double interp_duration);
  void InitializeRampToMin(const double interp_duration);

  void UpdateRampToMax(const double state_machine_time);
  void UpdateRampToMin(const double state_machine_time);

  void UpdateInstantToMax();
  void UpdateInstantToMin();

  Eigen::VectorXd GetWeight() {
    return task_->Weight();
  }


private:
  Task *task_;

  Eigen::VectorXd w_max_;
  Eigen::VectorXd w_min_;
  Eigen::VectorXd w_start_;
  Eigen::VectorXd w_fin_;

  double duration_;
};

#pragma once

class ForceTask {
public:
  ForceTask(const dim) : dim_(dim), rf_des_(Eigen::VectorXd::Zero(dim)){};
  virtual ~ForceTask();

  void UpdateDesired(const Eigen::VectorXd &rf_des) { rf_des_ = rf_des; };

  // getter
  const Eigen::VectorXd GetDesiredRF() { return rf_des_; }

protected:
  int dim_;
  Eigen::VectorXd rf_des_;
}

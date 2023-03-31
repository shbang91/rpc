#pragma once
#include <Eigen/Dense>

class QPParams;

class QPParamsManager {
public:
  QPParamsManager(QPParams *qp_params);
  ~QPParamsManager() = default;

  void InitializeWDeltaRfInterpolation(const Eigen::VectorXd &target_W_delta_fr,
                                       double duration);
  void UpdateWDeltaRfInterpolation(double query_time);

private:
  QPParams *qp_params_;
  Eigen::VectorXd init_W_delta_rf_;
  Eigen::VectorXd fin_W_delta_rf_;
  double duration_;
};

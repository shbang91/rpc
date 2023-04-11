#include "controller/whole_body_controller/managers/qp_params_manager.hpp"
#include "controller/whole_body_controller/wbic/wbic.hpp"
#include "util/util.hpp"

QPParamsManager::QPParamsManager(QPParams *qp_params)
    : qp_params_(qp_params), duration_(0.) {
  util::PrettyConstructor(2, "QPParamsManager");

  init_W_delta_rf_ = qp_params_->W_delta_rf_;
  fin_W_delta_rf_ = Eigen::VectorXd::Zero(init_W_delta_rf_.size());
  init_W_xc_ddot_ = qp_params_->W_xc_ddot_;
  fin_W_xc_ddot_ = Eigen::VectorXd::Zero(init_W_xc_ddot_.size());
}

void QPParamsManager::InitializeWDeltaRfInterpolation(
    const Eigen::VectorXd &target_W_delta_rf, double duration) {
  init_W_delta_rf_ = qp_params_->W_delta_rf_;
  fin_W_delta_rf_ = target_W_delta_rf;
  duration_ = duration;
}

void QPParamsManager::UpdateWDeltaRfInterpolation(double query_time) {
  query_time = util::Clamp(query_time, 0., duration_);

  Eigen::VectorXd W_delta_rf =
      init_W_delta_rf_ +
      (fin_W_delta_rf_ - init_W_delta_rf_) / duration_ * query_time;

  qp_params_->W_delta_rf_ = W_delta_rf;
}

void QPParamsManager::InitializeWContactInterpolation(
    const Eigen::VectorXd &target_W_xc_ddot, double duration) {
  init_W_xc_ddot_ = qp_params_->W_xc_ddot_;
  fin_W_xc_ddot_ = target_W_xc_ddot;
  duration_ = duration;
}
void QPParamsManager::UpdateWContactInterpolation(double query_time) {
  query_time = util::Clamp(query_time, 0., duration_);

  Eigen::VectorXd W_xc_ddot =
      init_W_xc_ddot_ +
      (fin_W_xc_ddot_ - init_W_xc_ddot_) / duration_ * query_time;

  qp_params_->W_xc_ddot_ = W_xc_ddot;
}

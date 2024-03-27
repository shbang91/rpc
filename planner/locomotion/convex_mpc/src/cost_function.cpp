#include "convex_mpc/cost_function.hpp"

CostFunction::CostFunction(const Matrix6d &Qqq, const Matrix6d &Qvv,
                           const Matrix6d &Quu, const double decay_rate,
                           const Matrix6d &Qqq_terminal,
                           const Matrix6d &Qvv_terminal)
    : Qqq_(Qqq), Qvv_(Qvv), Quu_(Matrix12d::Zero()), decay_rate_(decay_rate),
      Qqq_terminal_(Qqq_terminal), Qvv_terminal_(Qvv_terminal) {
  for (int i = 0; i < 2; ++i) {
    Quu_.template block<6, 6>(6 * i, 6 * i) = Quu;
  }
}

void CostFunction::initQP(QPData &qp_data) {
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    // running cost
    qp_data.qp_[i].Q.template topLeftCorner<6, 6>() =
        std::pow(decay_rate_, i) * Qqq_;
    qp_data.qp_[i].Q.template bottomRightCorner<6, 6>() =
        std::pow(decay_rate_, i) * Qvv_;
  }
  // terminal cost
  qp_data.qp_[qp_data.dim_.N].Q.template topLeftCorner<6, 6>() =
      std::pow(decay_rate_, qp_data.dim_.N) * Qqq_terminal_;
  qp_data.qp_[qp_data.dim_.N].Q.template bottomRightCorner<6, 6>() =
      std::pow(decay_rate_, qp_data.dim_.N) * Qvv_terminal_;
}

void CostFunction::setQP(const Eigen::VectorXd &init_state,
                         const aligned_vector<Vector12d> &des_state_traj,
                         QPData &qp_data) {
  for (int i = 0; i < qp_data.dim_.N; ++i) {
    // running cost
    qp_data.qp_[i].q.template head<6>().noalias() =
        -1. * Qqq_ * des_state_traj[i].head<6>() * std::pow(decay_rate_, i);
    qp_data.qp_[i].q.template tail<6>().noalias() =
        -1. * Qvv_ * des_state_traj[i].tail<6>() * std::pow(decay_rate_, i);
  }
  // terminal cost
  qp_data.qp_[qp_data.dim_.N].q.template head<6>() =
      -1. * Qqq_terminal_ * des_state_traj[qp_data.dim_.N].head<6>() *
      std::pow(decay_rate_, qp_data.dim_.N);
  qp_data.qp_[qp_data.dim_.N].q.template tail<6>() =
      -1. * Qvv_terminal_ * des_state_traj[qp_data.dim_.N].head<6>() *
      std::pow(decay_rate_, qp_data.dim_.N);

  for (int i = 0; i < qp_data.dim_.N; ++i) {
    qp_data.qp_[i].S.setZero();
    qp_data.qp_[i].R =
        Quu_.topLeftCorner(qp_data.dim_.nu[i], qp_data.dim_.nu[i]) *
        std::pow(decay_rate_, i);
    qp_data.qp_[i].r.setZero();
  }
}

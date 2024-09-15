#include "convex_mpc/qp_data.hpp"

void QPData::init(const int horizon_length) {
  // Here, we allocate memory as the possible maximum size.
  // ###########################
  // initialize QP variable size
  // ###########################
  const int N = horizon_length;
  dim_.resize(N);
  const int nx = 12;
  const int nu = 12; // possible max size
  // fill vector
  std::fill(dim_.nx.begin(), dim_.nx.end(), nx);
  std::fill(dim_.nu.begin(), dim_.nu.end(), nu);
  // fz_min < fz < fz_max for 2 legs
  std::fill(dim_.nbu.begin(), dim_.nbu.end(), 2);
  dim_.nbu[N] = 0;
  // -inf < cone * [mx, my, mz,fx, fy, fz,] < 0 for 2 legs
  std::fill(dim_.ng.begin(), dim_.ng.end(), 32);
  dim_.ng[N] = 0;

  // ###########################
  // initialize QP size
  // ###########################
  qp_.resize(N + 1);
  // dynamics
  for (int i = 0; i < dim_.N; ++i) {
    qp_[i].A.resize(dim_.nx[i], dim_.nx[i]);
    qp_[i].A.setZero();
    qp_[i].B.resize(dim_.nx[i], dim_.nu[i]);
    qp_[i].B.setZero();
    qp_[i].b.resize(dim_.nx[i]);
    qp_[i].b.setZero();
  }
  // cost
  for (int i = 0; i < dim_.N; ++i) {
    qp_[i].Q.resize(dim_.nx[i], dim_.nx[i]);
    qp_[i].Q.setZero();
    qp_[i].S.resize(dim_.nu[i], dim_.nx[i]);
    qp_[i].S.setZero();
    qp_[i].R.resize(dim_.nu[i], dim_.nu[i]);
    qp_[i].R.setZero();
    qp_[i].q.resize(dim_.nx[i]);
    qp_[i].q.setZero();
    qp_[i].r.resize(dim_.nu[i]);
    qp_[i].r.setZero();
  }
  qp_[N].Q.resize(dim_.nx[N], dim_.nx[N]);
  qp_[N].Q.setZero();
  qp_[N].q.resize(dim_.nx[N]);
  qp_[N].q.setZero();

  // box constraints
  for (int i = 0; i < dim_.N; ++i) {
    qp_[i].lbu.resize(dim_.nbu[i]);
    qp_[i].lbu.setZero();
    qp_[i].ubu.resize(dim_.nbu[i]);
    qp_[i].ubu.setZero();
    qp_[i].idxbu.clear();
    qp_[i].idxbu.resize(dim_.nbu[i]);
  }

  // inequality constraints
  for (int i = 0; i < dim_.N; ++i) {
    qp_[i].C.resize(dim_.ng[i], dim_.nx[i]);
    qp_[i].C.setZero();
    qp_[i].D.resize(dim_.ng[i], dim_.nu[i]);
    qp_[i].D.setZero();
    qp_[i].lg.resize(dim_.ng[i]);
    qp_[i].lg.setZero();
    qp_[i].ug.resize(dim_.ng[i]);
    qp_[i].ug.setZero();
    qp_[i].ug_mask.setZero(dim_.ng[i]);
  }

  // check qp dimension size
  dim_.checkSize(qp_);

  // ###########################
  // initialize qp solution size
  // ###########################
  qp_solution_.resize(N + 1);
  for (int i = 0; i < dim_.N; ++i) {
    qp_solution_[i].x.resize(nx);
    qp_solution_[i].u.resize(nu);
  }
  qp_solution_[N].x.resize(nx);
}

void QPData::resize(const std::vector<int> &num_contact_over_horizon) {
  // assume surface contact with wrench cone constraints
  for (int i = 0; i < dim_.N; ++i) {
    dim_.nu[i] = 6 * num_contact_over_horizon[i];
    dim_.nbu[i] = num_contact_over_horizon[i];
    dim_.ng[i] = 16 * num_contact_over_horizon[i];
  }
  // dynamics
  for (int i = 0; i < dim_.N; ++i) {
    qp_[i].B.resize(dim_.nx[i], dim_.nu[i]);
    qp_[i].B.setZero();
  }
  // cost
  for (int i = 0; i < dim_.N; ++i) {
    qp_[i].S.resize(dim_.nu[i], dim_.nx[i]);
    qp_[i].S.setZero();
    qp_[i].R.resize(dim_.nu[i], dim_.nu[i]);
    qp_[i].R.setZero();
    qp_[i].r.resize(dim_.nu[i]);
    qp_[i].r.setZero();
  }
  // constraints
  for (int i = 0; i < dim_.N; ++i) {
    qp_[i].lbu.resize(dim_.nbu[i]);
    qp_[i].lbu.setZero();
    qp_[i].ubu.resize(dim_.nbu[i]);
    qp_[i].ubu.setZero();
    qp_[i].idxbu.clear();
    qp_[i].idxbu.resize(dim_.nbu[i]);
  }
  for (int i = 0; i < dim_.N; ++i) {
    qp_[i].C.resize(dim_.ng[i], dim_.nx[i]);
    qp_[i].C.setZero();
    qp_[i].D.resize(dim_.ng[i], dim_.nu[i]);
    qp_[i].D.setZero();
    qp_[i].lg.resize(dim_.ng[i]);
    qp_[i].lg.setZero();
    qp_[i].ug.resize(dim_.ng[i]);
    qp_[i].ug.setZero();
    qp_[i].ug_mask.setZero(dim_.ng[i]);
  }

  // check qp dimension size
  dim_.checkSize(qp_);

  // qp solution size
  for (int i = 0; i < dim_.N; ++i) {
    qp_solution_[i].u.resize(dim_.nu[i]);
  }
}

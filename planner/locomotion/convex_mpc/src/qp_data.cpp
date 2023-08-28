#include "convex_mpc/qp_data.hpp"

namespace convexmpc {

void QPData::init(const ContactSchedule &contact_schedule) {
  // Here, we allocate memory as the possible maximum size.
  // initialize dim
  const int N = contact_schedule.N();
  dim_.resize(N);
  const int nx = 12;
  const int nu = 12; // possible max size
  // hpipm::fill_vector(dim_.nx, nx);
  // hpipm::fill_vector(dim_.nu, nu);
  // hpipm::fill_vector(dim_.nbx, 0);
  // dim_.nbx[0] = nx;
  // hpipm::fill_vector(dim_.nbu, 4); // fz_min < fz < fz_max for 4 legs
  // dim_.nbu[N] = 0;
  // hpipm::fill_vector(dim_.ng, 16); // -inf < cone * [fx, fy, fz] < 0 for 4
  // legs dim_.ng[N] = 0;

  // fill vector
  std::fill(dim_.nx.begin(), dim_.nx.end(), nx);
  std::fill(dim_.nu.begin(), dim_.nu.end(), nu);
  std::fill(dim_.nbx.begin(), dim_.nbx.end(), 0);
  dim_.nbx[0] = nx;
  std::fill(dim_.nbu.begin(), dim_.nbu.end(),
            4); // fz_min < fz < fz_max for 4 legs
  dim_.nbu[N] = 0;
  std::fill(dim_.ng.begin(), dim_.ng.end(),
            16); // -inf < cone * [fx, fy, fz] < 0 for 4 legs
  dim_.ng[N] = 0;
  // const auto dim_err_msg = dim_.checkSize();
  // if (!dim_err_msg.empty()) {
  // for (const auto &e : dim_err_msg) {
  // std::cout << e << std::endl;
  //}
  // return;
  //}
  // dim_.createHpipmData();

  // initialize qp
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
    // for (int j = 0; j < dim_.nbu[i]; ++j) {
    // qp_[i].idxbu.push_back(j);
    //}
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
    qp_[i].lg_mask.setZero(dim_.ng[i]);
  }
  // qp_.lg_mask.resize(dim_.N + 1);
  // qp_[0].lg_mask.resize(0);
  for (int i = 0; i < N; ++i) {
    qp_[i].lg_mask = Eigen::VectorXd::Zero(16);
  }
  // const auto qp_err_msg = qp_.checkSize(dim_);
  // if (!qp_err_msg.empty()) {
  // for (const auto &e : qp_err_msg) {
  // std::cout << e << std::endl;
  //}
  // return;
  //}
  // qp_.createHpipmData(dim_);

  // initialize qp solution
  qp_solution_.resize(N + 1);
  // const auto sol_err_msg = qp_solution_.checkSize(dim_);
  // if (!sol_err_msg.empty()) {
  // for (const auto &e : sol_err_msg) {
  // std::cout << e << std::endl;
  //}
  // return;
  //}
  // qp_solution_.createHpipmData(dim_);
}

void QPData::resize(const ContactSchedule &contact_schedule) {
  for (int i = 0; i < dim_.N; ++i) {
    dim_.nu[i] =
        3 * contact_schedule.numActiveContacts(contact_schedule.phase(i));
  }
  for (int i = 0; i < dim_.N; ++i) {
    dim_.nbu[i] =
        // 3 * contact_schedule.numActiveContacts(contact_schedule.phase(i));
        1 * contact_schedule.numActiveContacts(contact_schedule.phase(i));
  }
  for (int i = 0; i < dim_.N; ++i) {
    dim_.ng[i] =
        4 * contact_schedule.numActiveContacts(contact_schedule.phase(i));
  }
  // const auto dim_err_msg = dim_.checkSize();
  // if (!dim_err_msg.empty()) {
  // for (const auto &e : dim_err_msg) {
  // std::cout << e << std::endl;
  //}
  // return;
  //}
  // dim_.createHpipmData();

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
    // for (int j = 0; j < dim_.nbu[i]; ++j) {
    // qp_[i].idxbu.push_back(j);
    //}
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
    qp_[i].lg_mask.setZero(dim_.ng[i]);
  }
  // const auto qp_err_msg = qp_.checkSize(dim_);
  // if (!qp_err_msg.empty()) {
  // for (const auto &e : qp_err_msg) {
  // std::cout << e << std::endl;
  //}
  // return;
  //}
  // qp_.createHpipmData(dim_);

  for (int i = 0; i < dim_.N; ++i) {
    qp_solution_[i].u.resize(dim_.nu[i]);
  }
  // const auto sol_err_msg = qp_solution_.checkSize(dim_);
  // if (!sol_err_msg.empty()) {
  // for (const auto &e : sol_err_msg) {
  // std::cout << e << std::endl;
  //}
  // return;
  //}
  // qp_solution_.createHpipmData(dim_);

  // resize
  // dim_.resize(qp_);
}

bool QPData::checkSize() const {
  // const auto dim_err_msg = dim_.checkSize();
  // if (!dim_err_msg.empty()) {
  // for (const auto &e : dim_err_msg) {
  // std::cout << e << std::endl;
  //}
  // return false;
  //}
  // const auto qp_err_msg = qp_.checkSize(dim_);
  // if (!qp_err_msg.empty()) {
  // for (const auto &e : qp_err_msg) {
  // std::cout << e << std::endl;
  //}
  // return false;
  //}
  // const auto sol_err_msg = qp_solution_.checkSize(dim_);
  // if (!sol_err_msg.empty()) {
  // for (const auto &e : sol_err_msg) {
  // std::cout << e << std::endl;
  //}
  // return false;
  //}
  return true;
}

} // namespace convexmpc

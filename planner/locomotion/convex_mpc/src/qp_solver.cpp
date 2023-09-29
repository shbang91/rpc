#include "convex_mpc/qp_solver.hpp"

void QPSolver::init(QPData &qp_data) {
  // settings.createHpipmData(qp_data.dim);
  // solver_.createHpipmData(qp_data.dim, settings);

  // initialize solver here
  solver_ = std::make_shared<hpipm::OcpQpIpmSolver>(qp_data.qp_, settings);
}

void QPSolver::solve(const Vector12d &init_state, QPData &qp_data) {
  // settings_.createHpipmData(qp_data.dim);
  // solver_.createHpipmData(qp_data.dim, settings);
  // for (int i = 0; i < qp_data.dim_.N; ++i) {
  // std::cout << "A: " << std::endl;
  // std::cout << qp_data.qp_[i].A << std::endl;
  // std::cout << "B: " << std::endl;
  // std::cout << qp_data.qp_[i].B << std::endl;
  // std::cout << "C: " << std::endl;
  // std::cout << qp_data.qp_[i].C << std::endl;
  // std::cout << "D: " << std::endl;
  // std::cout << qp_data.qp_[i].D << std::endl;
  // std::cout << "lg_mask: " << std::endl;
  // std::cout << qp_data.qp_[i].lg_mask << std::endl;
  // std::cout << "idxbu: " << std::endl;
  // for (const auto &e : qp_data.qp_[i].idxbu) {
  // std::cout << e << "," << std::endl;
  //}
  //}
  const auto &solver_status =
      solver_->solve(init_state, qp_data.qp_, qp_data.qp_solution_);
  if (solver_status != hpipm::HpipmStatus::Success) {
    std::cout << hpipm::to_string(solver_status) << std::endl;
    const auto &stat = solver_->getSolverStatistics();
    std::cout << stat << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

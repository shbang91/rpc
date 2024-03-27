#include "convex_mpc/qp_solver.hpp"

void QPSolver::init(QPData &qp_data) {
  // initialize solver here
  solver_ = std::make_shared<hpipm::OcpQpIpmSolver>(qp_data.qp_, settings);
}

void QPSolver::solve(const Vector12d &init_state, QPData &qp_data) {
  const auto &solver_status =
      solver_->solve(init_state, qp_data.qp_, qp_data.qp_solution_);
  if (solver_status != hpipm::HpipmStatus::Success) {
    std::cout << hpipm::to_string(solver_status) << std::endl;
    const auto &stat = solver_->getSolverStatistics();
    std::cout << stat << std::endl;
    // std::exit(EXIT_FAILURE);
  }
}

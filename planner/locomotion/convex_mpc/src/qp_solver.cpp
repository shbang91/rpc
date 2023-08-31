#include "convex_mpc/qp_solver.hpp"

namespace convexmpc {

void QPSolver::init(QPData &qp_data) {
  // settings.createHpipmData(qp_data.dim);
  // solver_.createHpipmData(qp_data.dim, settings);

  // initialize solver here
  solver_ = std::make_shared<hpipm::OcpQpIpmSolver>(qp_data.qp_, settings);
}

void QPSolver::solve(const Eigen::VectorXd &init_state, QPData &qp_data) {
  // settings_.createHpipmData(qp_data.dim);
  // solver_.createHpipmData(qp_data.dim, settings);
  const auto &solver_status =
      solver_->solve(init_state, qp_data.qp_, qp_data.qp_solution_);
  if (solver_status != hpipm::HpipmStatus::Success) {
    std::cout << hpipm::to_string(solver_status) << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

} // namespace convexmpc

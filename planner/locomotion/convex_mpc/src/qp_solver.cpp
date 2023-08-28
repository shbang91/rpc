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
  std::cout << "====================================================="
            << std::endl;
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(12);
  std::cout << "x0: " << std::endl;
  std::cout << x0.transpose() << std::endl;
  for (int i = 0; i < 5; ++i) {
    x0 = qp_data.qp_[i].A * x0 + qp_data.qp_[i].b;
    std::cout << x0.transpose() << std::endl;
    // std::cout << "A:" << std::endl;
    // std::cout << qp_data.qp_[i].A << std::endl;
    // std::cout << "B:" << std::endl;
    // std::cout << qp_data.qp_[i].B << std::endl;
    // std::cout << "b:" << std::endl;
    // std::cout << qp_data.qp_[i].b << std::endl;
    // std::cout << "Q:" << std::endl;
    // std::cout << qp_data.qp_[i].Q << std::endl;
    // std::cout << "R:" << std::endl;
    // std::cout << qp_data.qp_[i].R << std::endl;
    // std::cout << "D:" << std::endl;
    // std::cout << qp_data.qp_[i].D << std::endl;
  }
  const auto &solver_status =
      solver_->solve(init_state, qp_data.qp_, qp_data.qp_solution_);
  if (solver_status != hpipm::HpipmStatus::Success) {
    std::cout << hpipm::to_string(solver_status) << std::endl;
    std::exit(EXIT_FAILURE);
  }
} // namespace convexmpc

} // namespace convexmpc

#ifndef CONVEX_MPC_QP_SOLVER_HPP_
#define CONVEX_MPC_QP_SOLVER_HPP_

#include "convex_mpc/qp_data.hpp"
#include "convex_mpc/types.hpp"
#include "hpipm-cpp/hpipm-cpp.hpp"

class QPSolver {
public:
  QPSolver() = default;

  ~QPSolver() = default;

  void init(QPData &qp_data);

  void solve(const Vector12d &init_state, QPData &qp_data);

  hpipm::OcpQpIpmSolverSettings settings;

private:
  std::shared_ptr<hpipm::OcpQpIpmSolver> solver_;
};

#endif // CONVEX_MPC_QP_DATA_HPP_

#ifndef CONVEX_MPC_SOLVER_OPTIONS_HPP_
#define CONVEX_MPC_SOLVER_OPTIONS_HPP_

#include <vector>

struct SolverOptions {
  int iter_max = 50; // max iter

  double alpha_min = 1e-8;

  double mu0 = 1.0e+02; // intial barrier parameter

  double tol_stat = 1.0e-04; // convergence criteria

  double tol_eq = 1.0e-04; // convergence criteria

  double tol_ineq = 1.0e-04; // convergence criteria

  double tol_comp = 1.0e-04; // convergence criteria

  double reg_prim = 1.0e-12; // reg

  int warm_start = 0; // use warm start or not

  int pred_corr = 1; // use correction step

  int ric_alg = 0; // use square-root Riccati or not

  int split_step =
      1; //  use different step for primal and dual variables or not

  // int iter_max = 30; // max iter

  // double alpha_min = 1e-12;

  // double mu0 = 1.0e+01; // intial barrier parameter

  // double tol_stat = 1.0e-06; // convergence criteria

  // double tol_eq = 1.0e-08; // convergence criteria

  // double tol_ineq = 1.0e-08; // convergence criteria

  // double tol_comp = 1.0e-08; // convergence criteria

  // double reg_prim = 1.0e-12; // reg

  // int warm_start = 0; // use warm start or not

  // int pred_corr = 1; // use correction step

  // int ric_alg = 0; // use square-root Riccati or not

  // int split_step =
  // 0; //  use different step for primal and dual variables or not
};

#endif // CONVEX_MPC_SOLVER_OPTIONS_HPP_

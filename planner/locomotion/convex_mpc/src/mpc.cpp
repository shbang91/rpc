#include "convex_mpc/mpc.hpp"

#include <cassert>

namespace convexmpc {

MPC::MPC(const int horizon_length, const double mpc_dt,
         const StateEquation &state_equation, const CostFunction &cost_function,
         const FrictionCone &friction_cone, const SolverOptions &solver_options)
    : horizon_length_(horizon_length), mpc_dt_(mpc_dt),
      initial_state_(Vector12d::Zero()), feet_pos_(Vector6d::Zero()),
      state_equation_(state_equation), cost_function_(cost_function),
      friction_cone_(friction_cone), qp_data_(), qp_solver_() {
  this->_setOptions(solver_options);
  this->_init();
}

void MPC::_setOptions(const SolverOptions &solver_options) {
  qp_solver_.settings.mode = hpipm::HpipmMode::Speed;
  qp_solver_.settings.iter_max = solver_options.iter_max;
  qp_solver_.settings.mu0 = solver_options.mu0;
  qp_solver_.settings.tol_stat = solver_options.tol_stat;
  qp_solver_.settings.tol_eq = solver_options.tol_eq;
  qp_solver_.settings.tol_ineq = solver_options.tol_ineq;
  qp_solver_.settings.tol_comp = solver_options.tol_comp;
  qp_solver_.settings.reg_prim = solver_options.reg_prim;
  qp_solver_.settings.warm_start = solver_options.warm_start;
  qp_solver_.settings.pred_corr = solver_options.pred_corr;
  qp_solver_.settings.ric_alg = solver_options.ric_alg;
  qp_solver_.settings.split_step = solver_options.split_step;
  qp_solver_.settings.alpha_min = solver_options.alpha_min;
}

void MPC::_init() {
  qp_data_.init(horizon_length_);
  qp_solver_.init(qp_data_);
  state_equation_.initQP(qp_data_);
  cost_function_.initQP(qp_data_);
  // assert(qp_data_.checkSize());
  mpc_solution_.init(horizon_length_, mpc_dt_);
}

void MPC::solve() {
  qp_data_.resize(num_contacts_vec_);
  state_equation_.setQP(initial_state_, contact_trajectory_, feet_pos_,
                        qp_data_);
  cost_function_.setQP(initial_state_, des_state_trajectory_, qp_data_);
  friction_cone_.setQP(qp_data_);
  qp_solver_.solve(initial_state_, qp_data_);
  // assert(qp_data_.checkSize());
  mpc_solution_.update(contact_trajectory_, qp_data_);
}

} // namespace convexmpc

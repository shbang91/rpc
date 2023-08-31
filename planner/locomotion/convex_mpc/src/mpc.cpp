#include "convex_mpc/mpc.hpp"

#include <cassert>

namespace convexmpc {

MPC::MPC(const StateEquation &state_equation, const CostFunction &cost_function,
         const FrictionCone &friction_cone)
    : state_equation_(state_equation), cost_function_(cost_function),
      friction_cone_(friction_cone), qp_data_(), qp_solver_() {}

void MPC::setOptions(const SolverOptions &solver_options) {
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

void MPC::init(const ContactSchedule &contact_schedule) {

  qp_data_.init(contact_schedule);
  qp_solver_.init(qp_data_);
  state_equation_.initQP(qp_data_);
  cost_function_.initQP(qp_data_);
  assert(qp_data_.checkSize());
  mpc_solution_.init(contact_schedule);
}

void MPC::solve(const Eigen::VectorXd &init_state,
                const RobotState &robot_state,
                const ContactSchedule &contact_schedule,
                const GaitCommand &gait_command) {
  qp_data_.resize(contact_schedule);
  state_equation_.setQP(contact_schedule, robot_state, qp_data_);
  cost_function_.setQP(init_state, contact_schedule, robot_state, gait_command,
                       qp_data_);
  // cost_function_.setQP(contact_schedule, robot_state, gait_command,
  // qp_data_);
  friction_cone_.setQP(qp_data_);
  qp_solver_.solve(init_state, qp_data_);
  assert(qp_data_.checkSize());
  mpc_solution_.update(contact_schedule, robot_state, qp_data_);
}

} // namespace convexmpc

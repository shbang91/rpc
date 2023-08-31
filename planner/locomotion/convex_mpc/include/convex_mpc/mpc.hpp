#ifndef CONVEX_MPC_MPC_HPP_
#define CONVEX_MPC_MPC_HPP_

#include "convex_mpc/contact_schedule.hpp"
#include "convex_mpc/cost_function.hpp"
#include "convex_mpc/friction_cone.hpp"
#include "convex_mpc/gait_command.hpp"
#include "convex_mpc/mpc_solution.hpp"
#include "convex_mpc/qp_data.hpp"
#include "convex_mpc/qp_solver.hpp"
#include "convex_mpc/robot_state.hpp"
#include "convex_mpc/solver_options.hpp"
#include "convex_mpc/state_equation.hpp"

namespace convexmpc {

class MPC {
public:
  MPC(const StateEquation &state_equation, const CostFunction &cost_function,
      const FrictionCone &friction_cone);

  MPC() = default;

  ~MPC() = default;

  void setOptions(const SolverOptions &solver_options);

  void init(const ContactSchedule &contact_schedule);

  void solve(const Eigen::VectorXd &init_state, const RobotState &robot_state,
             const ContactSchedule &contact_schedule,
             const GaitCommand &gait_command);

  const MPCSolution &getSolution() const { return mpc_solution_; }
  const QPData &getQPData() const { return qp_data_; }

private:
  StateEquation state_equation_;
  CostFunction cost_function_;
  FrictionCone friction_cone_;
  QPData qp_data_;
  QPSolver qp_solver_;
  MPCSolution mpc_solution_;
};

} // namespace convexmpc

#endif // CONVEX_MPC_MPC_HPP_

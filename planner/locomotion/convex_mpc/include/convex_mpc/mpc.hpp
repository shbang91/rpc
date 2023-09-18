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
  MPC(const int horizon_length, const double mpc_dt,
      const StateEquation &state_equation, const CostFunction &cost_function,
      const FrictionCone &friction_cone, const SolverOptions &solver_options);

  MPC() = default;

  ~MPC() = default;

  // use after calling setter functions
  void solve();

  // getter functions
  const MPCSolution &getSolution() const { return mpc_solution_; }
  const QPData &getQPData() const { return qp_data_; }

  // setter functions
  void setX0(const Eigen::Vector3d &rpy, const Eigen::Vector3d &com,
             const Eigen::Vector3d &ang_vel_world,
             const Eigen::Vector3d &lin_vel_world) {
    initial_state_.head<3>() = rpy;
    initial_state_.segment<3>(3) = com;
    initial_state_.segment<3>(6) = ang_vel_world;
    initial_state_.segment<3>(9) = lin_vel_world;
  }

  void setContactTrajectory(ContactState *contacts,
                            const std::size_t horizon_length) {
    contact_trajectory_.resize(horizon_length);
    num_contacts_vec_.resize(horizon_length);
    for (std::size_t i(0); i < horizon_length; i++) {
      contact_trajectory_[i] = contacts[i];
      int num_contact = 0;
      for (auto contact : contact_trajectory_[i].contact) {
        if (contact)
          num_contact++;
      }
      num_contacts_vec_[i] = num_contact;
    }
  }

  // TODO
  void
  setDesiredStateTrajectory(const aligned_vector<Vector12d> &des_state_traj) {
    des_state_trajectory_ = des_state_traj;
  }
  void setFeetRelativeToBody(const Vector6d &feet_pos) { feet_pos_ = feet_pos; }
  void setIntertiaTrajectory() {}

private:
  void _setOptions(const SolverOptions &solver_options);
  void _init();

  StateEquation state_equation_;
  CostFunction cost_function_;
  FrictionCone friction_cone_;
  QPData qp_data_;
  QPSolver qp_solver_;
  MPCSolution mpc_solution_;

  // mpc problem setting
  int horizon_length_;
  double mpc_dt_;

  // mpc inputs
  Vector12d initial_state_;
  Vector6d feet_pos_; // left, right order

  // mpc input trajectory
  std::vector<ContactState> contact_trajectory_;
  aligned_vector<Vector12d> des_state_trajectory_;

  // intermediate variables
  std::vector<int> num_contacts_vec_;
};

} // namespace convexmpc

#endif // CONVEX_MPC_MPC_HPP_

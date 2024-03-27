#ifndef CONVEX_MPC_COST_FUNCTION_HPP_
#define CONVEX_MPC_COST_FUNCTION_HPP_

#include <vector>

#include "convex_mpc/contact_schedule.hpp"
#include "convex_mpc/gait_command.hpp"
#include "convex_mpc/qp_data.hpp"
#include "convex_mpc/types.hpp"

#include "util/util.hpp"

class CostFunction {
public:
  CostFunction(const Matrix6d &Qqq, const Matrix6d &Qvv, const Matrix6d &Quu,
               const double decay_rate = 1.0,
               const Matrix6d &Qqq_terminal = Matrix6d::Identity(),
               const Matrix6d &Qvv_terminal = Matrix6d::Identity());

  CostFunction() = default;

  ~CostFunction() = default;

  void initQP(QPData &qp_data);

  void setQP(const Eigen::VectorXd &init_state,
             const aligned_vector<Vector12d> &des_state_traj, QPData &qp_data);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  double dt_;
  Matrix6d Qqq_, Qvv_;
  Matrix6d Qqq_terminal_, Qvv_terminal_;
  Matrix12d Quu_;
  double decay_rate_;
};

#endif // CONVEX_MPC_COST_FUNCTION_HPP_

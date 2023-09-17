#ifndef CONVEX_MPC_COST_FUNCTION_HPP_
#define CONVEX_MPC_COST_FUNCTION_HPP_

#include <vector>

#include "convex_mpc/contact_schedule.hpp"
#include "convex_mpc/gait_command.hpp"
#include "convex_mpc/qp_data.hpp"
#include "convex_mpc/robot_state.hpp"
#include "convex_mpc/single_rigid_body.hpp"
#include "convex_mpc/types.hpp"

#include "util/util.hpp"

namespace convexmpc {

class CostFunction {
public:
  // CostFunction(const double dt, const Matrix6d &Qqq, const Matrix6d &Qvv,
  // const Matrix3d &Quu, const double decay_rate = 1.0);
  CostFunction(const double dt, const Matrix6d &Qqq, const Matrix6d &Qvv,
               const Matrix6d &Quu, const double decay_rate = 1.0);

  CostFunction() = default;

  ~CostFunction() = default;

  void initQP(QPData &qp_data);

  void setQP(const ContactSchedule &contact_schedule,
             const RobotState &robot_state, const GaitCommand &gait_command,
             QPData &qp_data);

  void setQP(const Eigen::VectorXd &init_state, const GaitCommand &gait_command,
             QPData &qp_data);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  double dt_;
  Matrix6d Qqq_, Qvv_;
  Matrix12d Quu_;
  Vector6d v_command_, dq_command_; // vx, vy, vz, wx, wy, wz
  Vector7d base_pose_;
  aligned_vector<Vector7d> base_pose_ref_; // x, y, z, q.x, q.y, q.z, q.w order
  aligned_vector<Vector6d>
      base_pose_ref_euler_; // euler roll, pitch, yaw, x, y, z
  SingleRigidBody single_rigid_body_;
  Vector6d qdiff_;
  Matrix6d Jqdiff_, JtQqq_;
  double decay_rate_;
};

} // namespace convexmpc

#endif // CONVEX_MPC_COST_FUNCTION_HPP_

#include "convex_mpc/mpc_solution.hpp"

namespace convexmpc {

void MPCSolution::init(const ContactSchedule &contact_schedule) {
  const int N = contact_schedule.N();
  pos_ = aligned_vector<Vector3d>(N + 1, Vector3d::Zero());
  quat_ = aligned_vector<Quaterniond>(N + 1, Quaterniond::Identity());
  R_ = aligned_vector<Matrix3d>(N + 1, Matrix3d::Identity());
  pose_ = aligned_vector<Vector7d>(N + 1, Vector7d::Zero());
  twist_ = aligned_vector<Vector6d>(N + 1, Vector6d::Zero());
  v_ = aligned_vector<Vector3d>(N + 1, Vector3d::Zero());
  w_ = aligned_vector<Vector3d>(N + 1, Vector3d::Zero());
  f_ = aligned_vector<aligned_vector<Vector3d>>(
      N, aligned_vector<Vector3d>(4, Vector3d::Zero()));
}

void MPCSolution::update(const ContactSchedule &contact_schedule,
                         const RobotState &robot_state, const QPData &qp_data) {
  const int N = contact_schedule.N();
  const double dt = contact_schedule.dt();
  for (int i = 0; i <= N; ++i) {
    twist_[i] = qp_data.qp_solution_[i].x.template tail<6>();
    v_[i] = twist_[i].template tail<3>();
    w_[i] = twist_[i].template head<3>();
  }
  pose_[0].template head<4>() = robot_state.pose().template tail<4>();
  pose_[0].template tail<3>() = robot_state.pose().template head<3>();
  for (int i = 0; i < N; ++i) {
    single_rigid_body_.integrate(pose_[i], (dt * twist_[i]), pose_[i + 1]);
  }
  for (int i = 0; i < N + 1; ++i) {
    pos_[i] = pose_[i].template tail<3>();
    quat_[i].coeffs() = pose_[i].template head<4>();
    R_[i] = quat_[i].toRotationMatrix();
  }
  // contact_schedule
  for (int i = 0; i < N; ++i) {
    int nu = 0;
    for (int j = 0; j < 4; ++j) {
      const int phase = contact_schedule.phase(i);
      if (contact_schedule.isContactActive(phase)[j]) {
        f_[i][j] = qp_data.qp_solution_[i].u.template segment<3>(nu);
        nu += 3;
      } else {
        f_[i][j].setZero();
      }
    }
  }
}

} // namespace convexmpc

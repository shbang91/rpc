/*****************************************************************************
Copyright (c) 2019 MIT Biomimetic Robotics Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*****************************************************************************/

/*****************************************************************************
 Modified by Seung Hyeon Bang (bangsh0718@gmail.com) for humanoid case
*****************************************************************************/

#ifndef CONVEX_MPC_CONVEX_MPC_LOCOMOTION_HPP_
#define CONVEX_MPC_CONVEX_MPC_LOCOMOTION_HPP_

#include "controller/whole_body_controller/managers/cubic_beizer_trajectory_manager.hpp"
#include "convex_mpc/gait.hpp"
#include "convex_mpc/mpc.hpp"
#include "util/clock.hpp"
#include "util/interpolation.hpp"

#if B_USE_MATLOGGER
#include <matlogger2/matlogger2.h>
#include <matlogger2/utils/mat_appender.h>
#endif

/*****************************************************************************
 * Input
 * - Gait schedule
 * - Gait command (v_x, v_y, w_z)
 * Output: Locomotion policy
 * - Footstep
 * - Reaction Wrench
 * - Centroidal States
 *****************************************************************************/

class PinocchioRobotSystem;
class CompositeRigidBodyInertia;

namespace foot_side {
constexpr int LFoot = 0;
constexpr int RFoot = 1;
constexpr int NumFoot = 2;
} // namespace foot_side

struct MPCParams {
public:
  MPCParams() = default;
  ~MPCParams() = default;

  // cost penalizing term
  Vector6d Qqq_;
  Vector6d Qvv_;
  Vector6d Quu_;

  Vector6d Qqq_terminal_;
  Vector6d Qvv_terminal_;

  double decay_rate_;

  // nominal inertia
  Vector9d nominal_inertia_;

  // contact wrench cone constraints
  double mu_;
  double fz_min_;
  double fz_max_;
  double foot_half_length_;
  double foot_half_width_;
};

struct GaitParams {
public:
  GaitParams() = default;
  ~GaitParams() = default;

  // high level commands
  int gait_number_;
  double x_vel_cmd_;
  double y_vel_cmd_;
  double yaw_rate_cmd_;

  // swing foot
  double swing_height_;
  double raibert_gain_;
  double high_speed_turning_gain_;
  Eigen::Vector3d landing_foot_offset_;
};

class ConvexMPCLocomotion {
public:
  ConvexMPCLocomotion(const double dt, const int iterations_btw_mpc,
                      PinocchioRobotSystem *robot,
                      bool b_save_mpc_solution = false,
                      MPCParams *mpc_params = nullptr,
                      CompositeRigidBodyInertia *crbi = nullptr);
  ConvexMPCLocomotion() = default;
  ~ConvexMPCLocomotion() = default;

  void Initialize(const GaitCommand &gait_command,
                  const double des_body_height);
  void Solve();

  // setter
  void SetGait(const int gait_number) { gait_number_ = gait_number; };
  void SetSwingHeight(const double height) { swing_height_ = height; }
  void SetHipLocation(const aligned_vector<Vector3d> &base_to_hip_offset) {
    base_to_hip_offset_ = base_to_hip_offset;
  }
  void SetRaibertGain(const double raibert_gain) {
    raibert_gain_ = raibert_gain;
  }
  void SetHighSpeedTurningGain(const double high_speed_turning_gain) {
    high_speed_turning_gain_ = high_speed_turning_gain;
  }
  void SetFootLandingOffset(const Eigen::Vector3d &landing_foot_offset) {
    landing_foot_offset_ = landing_foot_offset;
  }

  // update robot specific WBO & WBO ang vel outside
  void UpdateWBO(const Eigen::Vector3d &wbo_ypr,
                 const Eigen::Vector3d &wbo_ang_vel) {
    wbo_ypr_ = wbo_ypr;
    wbo_ang_vel_ = wbo_ang_vel;
  }

  // MPC desired state trajectory
  aligned_vector<Vector12d> des_state_traj_;
  std::vector<aligned_vector<Eigen::Vector3d>> des_foot_pos_traj_;
  std::vector<aligned_vector<Eigen::Vector3d>> des_foot_ori_traj_;

  // WBC task variables
  // from Raibert Heuristics
  Eigen::Vector3d des_foot_pos_[foot_side::NumFoot]; // left, right order
  Eigen::Vector3d des_foot_vel_[foot_side::NumFoot];
  Eigen::Vector3d des_foot_acc_[foot_side::NumFoot];

  Eigen::Quaterniond des_foot_ori_[foot_side::NumFoot];
  Eigen::Vector3d des_foot_ang_vel_[foot_side::NumFoot];
  Eigen::Vector3d des_foot_ang_acc_[foot_side::NumFoot];

  // from desired state commands
  Eigen::Vector3d des_body_pos_;
  Eigen::Vector3d des_body_vel_;
  Eigen::Vector3d des_body_rpy_;
  Eigen::Vector3d des_body_ang_vel_;

  // from convex mpc
  Vector6d des_lf_wrench_;
  Vector6d des_rf_wrench_;

  Vector6d mpc_lf_wrench_cmd_first_;
  Vector6d mpc_lf_wrench_cmd_second_;
  Vector6d mpc_rf_wrench_cmd_first_;
  Vector6d mpc_rf_wrench_cmd_second_;

  // contact variable
  // from gait scheduler
  Eigen::Vector2d contact_state_;
  Eigen::Vector2d swing_state_;

  double x_vel_cmd_; // from yaml
  double y_vel_cmd_; // from yaml
  double yaw_rate_cmd_;

private:
  // robot states
  PinocchioRobotSystem *robot_;

  // mpc variables
  double dt_;
  int iterations_btw_mpc_;
  int n_horizon_;
  double dt_mpc_;

  void _InitializeConvexMPC(MPCParams *mpc_params);
  void _InitializeConvexMPC(); // TODO: to be deprecated
  void _SolveConvexMPC(int *contact_schedule_table);

  std::shared_ptr<CostFunction> cost_function_;
  std::shared_ptr<StateEquation> state_equation_;
  std::shared_ptr<FrictionCone> friction_cone_;
  std::shared_ptr<SolverOptions> solver_options_;
  std::shared_ptr<MPC> convex_mpc_;

  bool b_first_visit_ = true;
  bool b_first_visit_inertia_gen_ = true;
  int iteration_counter_;

  bool b_save_mpc_solution_;

  // gaits
  void _SetupBodyCommand();
  double x_vel_des_ = 0.0;
  double y_vel_des_ = 0.0;
  double yaw_rate_des_ = 0.0;
  double roll_des_ = 0.0;
  double pitch_des_ = 0.0;
  double yaw_des_ = 0.0;

  OffsetDurationGait standing_, walking_;
  OffsetDurationGait gait_for_inertia_;

  Eigen::Vector3d rpy_int_;
  Eigen::Vector3d rpy_comp_;

  Eigen::Vector3d wbo_ypr_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d wbo_ang_vel_ = Eigen::Vector3d::Zero();

  int gait_number_;
  int current_gait_number_ = gait::kStand;

  // state trajectory
  Vector6d stand_traj_;

  // varying inertia calculation
  Vector6d stand_base_traj_; // for base trajectory
  Eigen::Vector3d des_base_com_pos_in_world_;

  // reference trajectory generation
  Eigen::Vector3d des_com_vel_in_body_;

  // desired body states for WBC update
  Eigen::Vector3d des_com_pos_in_world_;
  Eigen::Vector3d des_com_vel_in_world_;
  double des_body_height_;

  // swing foot
  bool b_first_swing_[foot_side::NumFoot];
  double swing_height_;
  double raibert_gain_;
  double high_speed_turning_gain_;
  Eigen::Vector3d landing_foot_offset_;
  aligned_vector<Vector3d> foot_pos_;           // lfoot, rfoot order
  aligned_vector<Matrix3d> foot_ori_;           // lfoot, rfoot order
  aligned_vector<Vector3d> base_to_hip_offset_; // lfoot, rfoot order
  CubicBeizerTrajectoryManager<double>
      foot_swing_pos_trajectory_[foot_side::NumFoot];
  HermiteQuaternionCurve2 foot_swing_ori_trajectory_[foot_side::NumFoot];
  double swing_time_[foot_side::NumFoot];
  double swing_time_remaining_[foot_side::NumFoot];
  Eigen::Vector2d swing_states_;
  Eigen::Vector2d contact_states_;

  // for inertia traj
  CubicBeizerTrajectoryManager<double>
      foot_swing_pos_trajectory_for_inertia_[foot_side::NumFoot];
  HermiteQuaternionCurve2
      foot_swing_ori_trajectory_for_inertia_[foot_side::NumFoot];

  // clock
  Clock clock_;

  // composite rigid body inertia
  CompositeRigidBodyInertia *crbi_;

#if B_USE_MATLOGGER
  XBot::MatLogger2::Ptr logger_;
  XBot::MatAppender::Ptr appender_;
#endif
};

#endif // CONVEX_MPC_CONVEX_MPC_LOCOMOTION_HPP_

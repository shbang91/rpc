#ifndef CONVEX_MPC_CONVEX_MPC_LOCOMOTION_HPP_
#define CONVEX_MPC_CONVEX_MPC_LOCOMOTION_HPP_

#include "controller/whole_body_controller/managers/cubic_beizer_trajectory_manager.hpp"
#include "convex_mpc/gait.hpp"
#include "convex_mpc/mpc.hpp"

/**
 * Input: Gait schedule, Gait command
 * Output: Locomotion policy (footstep, reaction wrench, centroidal states)
 */

class PinocchioRobotSystem;

class ConvexMPCLocomotion {
public:
  ConvexMPCLocomotion(const double dt, const int iterations_btw_mpc,
                      PinocchioRobotSystem *robot);
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

  // WBC task variables
  // from Raibert Heuristics
  Eigen::Vector3d des_foot_pos_[2]; // left, right order
  Eigen::Vector3d des_foot_vel_[2];
  Eigen::Vector3d des_foot_acc_[2];

  // from desired state commands
  Eigen::Vector3d des_body_pos_;
  Eigen::Vector3d des_body_vel_;
  Eigen::Vector3d des_body_rpy_;
  Eigen::Vector3d des_body_ang_vel_;

  // from convex mpc
  Vector6d des_lf_wrench_;
  Vector6d des_rf_wrench_;

  // contact variable
  // from gait scheduler
  Eigen::Vector2d contact_state_;

private:
  // robot states
  PinocchioRobotSystem *robot_;

  // mpc variables
  double dt_;
  int iterations_btw_mpc_;
  int n_horizon_;
  double dt_mpc_;

  void _InitializeConvexMPC();
  void _SolveConvexMPC(int *contact_schedule_table);

  std::shared_ptr<CostFunction> cost_function_;
  std::shared_ptr<StateEquation> state_equation_;
  std::shared_ptr<FrictionCone> friction_cone_;
  std::shared_ptr<SolverOptions> solver_options_;
  std::shared_ptr<MPC> convex_mpc_;

  bool b_first_visit_ = true;
  int iteration_counter_;

  // gaits
  void _SetupBodyCommand();
  double x_vel_des_ = 0.0;
  double y_vel_des_ = 0.0;
  double yaw_rate_des_ = 0.0;
  double roll_des_ = 0.0;
  double pitch_des_ = 0.0;
  double yaw_des_ = 0.0;

  double x_vel_cmd_; // from yaml
  double y_vel_cmd_; // from yaml
  double yaw_rate_cmd_;

  OffsetDurationGait standing_, walking_;

  Eigen::Vector3d rpy_int_;
  Eigen::Vector3d rpy_comp_;

  int gait_number_;
  int current_gait_number_ = gait::kStand;

  // state trajectory
  Vector6d stand_traj_;

  // desired body states for WBC update
  Eigen::Vector3d des_body_pos_in_world_;
  Eigen::Vector3d des_body_vel_in_world_;
  double des_body_height_;

  // swing foot
  bool b_first_swing_[2];
  double swing_height_;
  aligned_vector<Vector3d> foot_pos_;           // lfoot, rfoot order
  aligned_vector<Vector3d> base_to_hip_offset_; // lfoot, rfoot order
  CubicBeizerTrajectoryManager<double> foot_swing_trajectory_[2];
  double swing_time_[2];
  double swing_time_remaining_[2];
};

#endif // CONVEX_MPC_CONVEX_MPC_LOCOMOTION_HPP_

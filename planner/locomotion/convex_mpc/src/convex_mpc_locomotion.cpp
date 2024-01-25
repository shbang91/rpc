#include "convex_mpc/convex_mpc_locomotion.hpp"
#include "configuration.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "util/util.hpp"

#include <cmath>

ConvexMPCLocomotion::ConvexMPCLocomotion(const double dt,
                                         const int iterations_btw_mpc,
                                         PinocchioRobotSystem *robot)
    : dt_(dt), iterations_btw_mpc_(iterations_btw_mpc), n_horizon_(10),
      robot_(robot), iteration_counter_(0),
      standing_(n_horizon_, Eigen::Vector2i(0, 0), Eigen::Vector2i(10, 10),
                "standing"),
      walking_(n_horizon_, Eigen::Vector2i(0, 5), Eigen::Vector2i(5, 5),
               "walking") {
  util::PrettyConstructor(2, "ConvexMPCLocomotion");

  rpy_int_.setZero();
  rpy_comp_.setZero();

  // convex mpc formulation
  _InitializeConvexMPC();

  // swing foot
  foot_pos_.resize(2);
  foot_ori_.resize(2);
  for (int i = 0; i < 2; ++i) {
    b_first_swing_[i] = true;
    foot_pos_[i].setZero();
    foot_ori_[i].setIdentity();
    des_foot_pos_[i].setZero();
    des_foot_vel_[i].setZero();
    des_foot_acc_[i].setZero();
    des_foot_ori_[i].setIdentity();
    des_foot_ang_vel_[i].setZero();
    des_foot_ang_acc_[i].setZero();
  }
  des_lf_wrench_.setZero();
  des_rf_wrench_.setZero();
  // double robot_total_weight = robot_->GetTotalWeight();
  // des_lf_wrench_[5] = robot_total_weight / 2.;
  // des_rf_wrench_[5] = robot_total_weight / 2.;
}

void ConvexMPCLocomotion::Initialize(const GaitCommand &gait_command,
                                     const double des_body_height) {
  x_vel_cmd_ = gait_command.vel_xy_des[0];
  y_vel_cmd_ = gait_command.vel_xy_des[1];
  yaw_rate_cmd_ = gait_command.yaw_rate;

  des_body_height_ = des_body_height;

  // swing foot
  for (int i = 0; i < 2; ++i) {
    b_first_swing_[i] = true;
  }

  b_first_visit_ = true;
}

void ConvexMPCLocomotion::Solve() {
  // body command setup
  _SetupBodyCommand();

  // check if transitioning to standing --> for later use
  if (gait_number_ == gait::kStand && current_gait_number_ != gait::kStand ||
      b_first_visit_) {
    stand_traj_[0] = 0.0;                        // roll
    stand_traj_[1] = 0.0;                        // pitch
    stand_traj_[2] = robot_->GetBodyOriYPR()[0]; // yaw
    // stand_traj_[3] = robot_->GetBodyPos()[0];    // TODO:base CoM x
    // stand_traj_[4] = robot_->GetBodyPos()[1];    // TODO:base CoM y
    stand_traj_[3] = robot_->GetRobotComPos()[0];
    stand_traj_[4] = robot_->GetRobotComPos()[1];
    stand_traj_[5] =
        des_body_height_; // des height should be same as stand up height

    des_body_pos_in_world_[0] = stand_traj_[3];
    des_body_pos_in_world_[1] = stand_traj_[4];
  }

  // pick gait
  Gait *gait = &standing_;
  if (gait_number_ == gait::kStand)
    gait = &standing_;
  else if (gait_number_ == gait::kWalking)
    gait = &walking_;
  current_gait_number_ = gait_number_;

  // TODO:integrate position setpoint
  Eigen::Vector3d des_body_vel_in_body(x_vel_des_, y_vel_des_,
                                       0.0); // in local body frame
  des_body_vel_in_world_ = robot_->GetBodyOriRot() * des_body_vel_in_body;
  Eigen::Vector3d body_vel_in_world =
      robot_->GetLinkSpatialVel(robot_->GetRootFrameName()).tail<3>();
  // robot_->GetRobotComLinVel();

  // integral-esque pitch and roll compensation from MIT Cheetah
  if (std::fabs(body_vel_in_world[0]) > 0.2)
    rpy_int_[1] +=
        dt_ * (pitch_des_ - robot_->GetBodyOriYPR()[1]) / body_vel_in_world[0];
  if (std::fabs(body_vel_in_world[1]) > 0.1)
    rpy_int_[0] +=
        dt_ * (roll_des_ - robot_->GetBodyOriYPR()[2]) / body_vel_in_world[1];

  rpy_int_[0] = fmin(fmax(rpy_int_[0], -0.25), 0.25);
  rpy_int_[1] = fmin(fmax(rpy_int_[1], -0.25), 0.25);
  rpy_comp_[0] = body_vel_in_world[1] * rpy_int_[0];
  rpy_comp_[1] = body_vel_in_world[0] * rpy_int_[1];

  // current foot pos
  for (int i = 0; i < 2; ++i) {
    foot_pos_[i] = robot_->GetBodyPos() +
                   robot_->GetBodyOriRot() *
                       robot_->GetLocomotionControlPointsIsometryInBody(i)
                           .translation(); // TODO: check both length are the
                                           // same at the very beginning
    foot_ori_[i] = robot_->GetBodyOriRot() *
                   robot_->GetLocomotionControlPointsIsometryInBody(i).linear();
  }

  // set body states desired
  if (gait != &standing_)
    des_body_pos_in_world_ +=
        dt_ * Eigen::Vector3d(des_body_vel_in_world_[0],
                              des_body_vel_in_world_[1], 0.0);

  // first visit initialization
  if (b_first_visit_) {
    // for body pos task
    des_body_pos_in_world_[0] =
        // robot_->GetBodyPos()[0]; // TODO: com xy vs body com xy
        robot_->GetRobotComPos()[0]; // TODO: com xy vs body com xy
    des_body_pos_in_world_[1] =
        // robot_->GetBodyPos()[1]; // TODO: com xy vs body com xy
        robot_->GetRobotComPos()[1]; // TODO: com xy vs body com xy

    for (int i = 0; i < 2; ++i) {
      // initialize swing foot trajectory
      foot_swing_trajectory_[i].SetInitialPosition(foot_pos_[i]);
      foot_swing_trajectory_[i].SetFinalPosition(foot_pos_[i]);
      foot_swing_trajectory_[i].SetHeight(swing_height_);
    }

    // b_first_visit_ = false;
  }

  // TODO:foot placement with raibert hueristic
  for (int leg = 0; leg < 2; ++leg) {
    swing_time_[leg] = gait->getSwingDuration(dt_mpc_, leg);
  }

  /*
  // TODO: figure what do these variables mean?
  double side_sign[2] = {-1, 1};
  double interleave_y[2] = {-0.08, 0.08};
  // double interleave_gain = -0.13;
  double interleave_gain = -0.2;
  // double v_abs = std::fabs(robot_->GetBodyVel[0]);
  double v_abs = std::fabs(des_body_vel_in_body[0]);
  */

  // foot placement strategy
  double side_sign[2] = {1, -1};
  for (int leg = 0; leg < 2; ++leg) {
    if (b_first_swing_[leg])
      swing_time_remaining_[leg] = swing_time_[leg];
    else
      swing_time_remaining_[leg] -= dt_;

    // TODO: make this as yaml (foot step offset)
    // Eigen::Vector3d offset(0.0, side_sign[leg] * 0.06, 0.0);
    Eigen::Vector3d offset(0.0, side_sign[leg] * 0.0, 0.0);

    double stance_time = gait->getStanceDuration(dt_mpc_, leg);
    Eigen::Vector3d foot_pos_from_body = base_to_hip_offset_[leg] + offset;
    Eigen::Vector3d foot_yaw_corrected =
        util::CoordinateRotation(util::CoordinateAxis::Z,
                                 yaw_rate_des_ * stance_time / 2.) *
        // util::CoordinateRotation(util::CoordinateAxis::Z,
        //-yaw_rate_des_ * stance_time / 2.) *
        foot_pos_from_body;

    Eigen::Vector3d des_vel;
    des_vel[0] = x_vel_des_;
    des_vel[1] = y_vel_des_;
    des_vel[2] = 0.0;

    Eigen::Vector3d des_foot_pos =
        robot_->GetBodyPos() +
        robot_->GetBodyOriRot() *
            (foot_yaw_corrected + des_vel * swing_time_remaining_[leg]);

    double p_rel_max = 0.4;
    // Using the estimated velocity is correct
    double pfx_rel = body_vel_in_world[0] * 0.5 * stance_time +
                     0.1 * (body_vel_in_world[0] - des_body_vel_in_world_[0]) +
                     (0.5 * robot_->GetBodyPos()[2] / 9.81) *
                         (body_vel_in_world[1] * yaw_rate_des_);

    float pfy_rel = body_vel_in_world[1] * 0.5 * stance_time +
                    0.1 * (body_vel_in_world[1] - des_body_vel_in_world_[1]) +
                    (0.5 * robot_->GetBodyPos()[2] / 9.81) *
                        (-body_vel_in_world[0] * yaw_rate_des_);
    pfx_rel = fmin(fmax(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fmin(fmax(pfy_rel, -p_rel_max), p_rel_max);
    des_foot_pos[0] += pfx_rel;
    des_foot_pos[1] += pfy_rel;
    // des_foot_pos[2] = -0.003;
    des_foot_pos[2] = 0.0;
    foot_swing_trajectory_[leg].SetFinalPosition(des_foot_pos);
  }

  // calculate gait
  gait->setIterations(iterations_btw_mpc_, iteration_counter_);
  // mpc iteration counter
  // iteration_counter_++;

  Eigen::Vector2d contact_states = gait->getContactState();
  Eigen::Vector2d swing_states = gait->getSwingState();
  int *contact_schedule_table = gait->getMPCGait();

  // solve convex mpc with fixed control frequency
  // if (iteration_counter_ % iterations_btw_mpc_ == 0)
  if (iteration_counter_ % 5 == 0)
    _SolveConvexMPC(contact_schedule_table);

  // contact state for state estimator
  Eigen::Vector2d se_contact_state(0.0, 0.0);

  // create foot trajectory
  for (int foot(0); foot < 2; foot++) {
    double contact_state = contact_states[foot];
    double swing_state = swing_states[foot];

    // TODO:clean this mess
    if (b_first_visit_) {
      if (foot == 0) {
        contact_state = 0.5;
        swing_state = 0.0;
      } else {
        contact_state = 0.0;
        swing_state = 0.001;
        b_first_visit_ = false;
      }
    }

    if (swing_state > 0) {
      // foot is in swing
      if (b_first_swing_[foot]) {
        b_first_swing_[foot] = false;
        foot_swing_trajectory_[foot].SetInitialPosition(foot_pos_[foot]);
        Eigen::Matrix3d world_R_body_yaw = robot_->GetBodyYawRotationMatrix();
        Eigen::Matrix3d des_foot_ori =
            util::CoordinateRotation(
                util::CoordinateAxis::Z,
                yaw_rate_des_ * gait->getStanceDuration(dt_mpc_, foot) / 2.0) *
            world_R_body_yaw;
        foot_swing_ori_trajectory_[foot].Initialize(
            Eigen::Quaterniond(foot_ori_[foot]).normalized(),
            Eigen::Vector3d::Zero(),
            Eigen::Quaterniond(des_foot_ori).normalized(),
            Eigen::Vector3d::Zero());
      }

      foot_swing_trajectory_[foot].ComputeSwingTrajectoryBezier(
          swing_state, swing_time_[foot]);

      // save for WBC foot task
      des_foot_pos_[foot] = foot_swing_trajectory_[foot].GetPosition();
      des_foot_vel_[foot] = foot_swing_trajectory_[foot].GetVelocity();
      des_foot_acc_[foot] = foot_swing_trajectory_[foot].GetAcceleration();

      des_foot_ori_[foot] =
          foot_swing_ori_trajectory_[foot].GetOrientation(swing_state);
      des_foot_ang_vel_[foot] =
          foot_swing_ori_trajectory_[foot].GetAngularVelocity(swing_state);
      des_foot_ang_acc_[foot] =
          foot_swing_ori_trajectory_[foot].GetAngularAcceleration(swing_state);

      // std::cout << "=================================================="
      //<< std::endl;
      // std::cout << "foot: " << foot << std::endl;
      // std::cout << "curr foot pos: " << foot_pos_[foot].transpose()
      //<< std::endl;
      // std::cout << "des foot pos: " << des_foot_pos_[foot].transpose()
      //<< std::endl;
      // std::cout << "des foot vel: " << des_foot_vel_[foot].transpose()
      //<< std::endl;
      // std::cout << "des foot acc: " << des_foot_acc_[foot].transpose()
      //<< std::endl;

      // TODO: misc (think about how to use this)
      Eigen::Vector3d des_leg_pos =
          robot_->GetBodyOriRot().transpose() *
              (des_foot_pos_[foot] - robot_->GetBodyPos()) -
          base_to_hip_offset_[foot];
      Eigen::Vector3d des_leg_vel = robot_->GetBodyOriRot().transpose() *
                                    (des_foot_vel_[foot] - body_vel_in_world);
      contact_state_[foot] = 0.0;
    } else {
      // foot is in contact
      b_first_swing_[foot] = true;

      // save for WBC foot task
      des_foot_pos_[foot] = foot_swing_trajectory_[foot].GetPosition();
      des_foot_vel_[foot] = foot_swing_trajectory_[foot].GetVelocity();
      des_foot_acc_[foot] = foot_swing_trajectory_[foot].GetAcceleration();

      // TODO: misc (think about how to use this)
      Eigen::Vector3d des_leg_pos =
          robot_->GetBodyOriRot().transpose() *
              (des_foot_pos_[foot] - robot_->GetBodyPos()) -
          base_to_hip_offset_[foot];
      Eigen::Vector3d des_leg_vel = robot_->GetBodyOriRot().transpose() *
                                    (des_foot_vel_[foot] - body_vel_in_world);

      se_contact_state[foot] = contact_state;
      // wbc update
      contact_state_[foot] = contact_state;
    }
  }

  // exit(0);

  // TODO: set contact state for state estimator (donno if this is needed)
  // state_estimator->SetCotnactPhase(se_contact_state);

  // saving for WBC
  // centroidal task update
  des_body_pos_[0] = des_body_pos_in_world_[0];
  des_body_pos_[1] = des_body_pos_in_world_[1];
  des_body_pos_[2] = des_body_height_;

  des_body_vel_[0] = des_body_vel_in_world_[0];
  des_body_vel_[1] = des_body_vel_in_world_[1];
  des_body_vel_[2] = 0.0;

  des_body_rpy_[0] = 0.0;
  des_body_rpy_[1] = 0.0;
  des_body_rpy_[2] = yaw_des_;

  // TODO(SH): check this!!
  // this is in body frame and assuming Euler-angle rate matrix is identity
  // (roll, pitch -> 0)
  des_body_ang_vel_[0] = 0.0;
  des_body_ang_vel_[1] = 0.0;
  des_body_ang_vel_[2] = yaw_rate_des_;

  // contact state
  // contact_state_ = gait->getContactState();

  // mpc iteration counter
  iteration_counter_++;
}

void ConvexMPCLocomotion::_InitializeConvexMPC() {
  dt_mpc_ = dt_ * iterations_btw_mpc_;

  // cost
  Eigen::MatrixXd Qqq = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Qvv = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Quu = Eigen::MatrixXd::Zero(6, 6);

  Qqq(0, 0) = 1000;
  Qqq(1, 1) = 1000;
  Qqq(2, 2) = 1000;
  Qqq(3, 3) = 1000;
  Qqq(4, 4) = 1000;
  Qqq(5, 5) = 1000;

  Qvv(0, 0) = 10;
  Qvv(1, 1) = 10;
  Qvv(2, 2) = 10;
  Qvv(3, 3) = 10;
  Qvv(4, 4) = 10;
  Qvv(5, 5) = 10;

  Quu(0, 0) = 1e-6;
  Quu(1, 1) = 1e-6;
  Quu(2, 2) = 1e-6;
  Quu(3, 3) = 1e-6;
  Quu(4, 4) = 1e-6;
  Quu(5, 5) = 1e-6;

  cost_function_ =
      std::make_shared<CostFunction>(Qqq, Qvv, Quu); // TODO: make yaml

  // state equation
  Eigen::Matrix3d nominal_inertia; // TODO: this params should be set outside
  nominal_inertia << 4.51486, -0.00153135, 0.885743, -0.00153135, 4.1509,
      -0.00292598, 0.885743, -0.00292598, 1.23588;
  state_equation_ = std::make_shared<StateEquation>(
      dt_mpc_, robot_->GetTotalMass(),
      nominal_inertia); // TODO:nominal inertia need to change for inertia
                        // roll-out

  // wrnech cone constraint (TODO: this params should be set outside)
  double mu = 0.5;
  double fz_min = 20.0;
  double fz_max = 500.0;
  double foot_half_length = 0.08;
  double foot_half_width = 0.04;
  friction_cone_ = std::make_shared<FrictionCone>(
      mu, fz_min, fz_max, foot_half_length, foot_half_width);

  // set solver options
  solver_options_ = std::make_shared<SolverOptions>();

  // convex mpc
  convex_mpc_ =
      std::make_shared<MPC>(n_horizon_, dt_mpc_, *state_equation_,
                            *cost_function_, *friction_cone_, *solver_options_);
}

void ConvexMPCLocomotion::_SolveConvexMPC(int *contact_schedule_table) {
  // TODO: solve mpc with contact schedule

  //=====================================================
  // set state trajectory
  //=====================================================
  // set body inertia
  Eigen::Matrix3d I_global = robot_->GetIg().topLeftCorner<3, 3>();
  Eigen::Matrix3d global_R_local = robot_->GetBodyOriRot();
  Eigen::Matrix3d I_local =
      global_R_local.transpose() * I_global * global_R_local;
  state_equation_->setBodyInertia(I_local);

  aligned_vector<Vector12d> des_state_traj;
  des_state_traj.resize(n_horizon_ + 1);

  Vector12d initial_state;
  initial_state.setZero();

  if (current_gait_number_ == gait::kStand) {
    // when standing

    // initial state
    initial_state.head<3>() =
        Eigen::Vector3d(roll_des_, pitch_des_, stand_traj_[2]); // rpy
    initial_state.segment<3>(3) =
        Eigen::Vector3d(stand_traj_[3], stand_traj_[4], stand_traj_[5]); // xyz

    for (int i = 0; i < n_horizon_ + 1; ++i) {
      // roll out desired trajectory
      des_state_traj[i] = initial_state;
    }
  } else {
    // when not standing

    // initial state compensation strategy
    // Eigen::Vector3d curr_body_pos_in_world = robot_->GetBodyPos();
    Eigen::Vector3d curr_body_pos_in_world = robot_->GetRobotComPos();

    const double max_pos_error = 0.1;
    double x_start = des_body_pos_in_world_[0];
    double y_start = des_body_pos_in_world_[1];

    if (x_start - curr_body_pos_in_world[0] > max_pos_error)
      x_start = curr_body_pos_in_world[0] + max_pos_error;
    if (curr_body_pos_in_world[0] - x_start > max_pos_error)
      x_start = curr_body_pos_in_world[0] - max_pos_error;

    if (y_start - curr_body_pos_in_world[1] > max_pos_error)
      y_start = curr_body_pos_in_world[1] + max_pos_error;
    if (curr_body_pos_in_world[1] - y_start > max_pos_error)
      y_start = curr_body_pos_in_world[1] - max_pos_error;

    // save des xy pos for wbc
    des_body_pos_in_world_[0] = x_start;
    des_body_pos_in_world_[1] = y_start;

    // initial state
    initial_state.head<3>() =
        Eigen::Vector3d(rpy_comp_[0], rpy_comp_[1], yaw_des_);
    initial_state.segment<3>(3) =
        Eigen::Vector3d(x_start, y_start, des_body_height_);
    initial_state.segment<3>(6) = Eigen::Vector3d(0.0, 0.0, yaw_rate_cmd_);
    initial_state.segment<3>(9) = Eigen::Vector3d(
        des_body_vel_in_world_[0], des_body_vel_in_world_[1], 0.0);

    for (int i = 0; i < n_horizon_ + 1; ++i) {
      // roll out desired trajectory
      des_state_traj[i] = initial_state;

      if (i == 0) {
        // start at current position
        des_state_traj[i](2) = robot_->GetBodyOriYPR()[0];
      } else {
        des_state_traj[i](2) =
            des_state_traj[i - 1](2) + dt_mpc_ * yaw_rate_cmd_;
        des_state_traj[i](3) =
            des_state_traj[i - 1](3) + dt_mpc_ * des_body_vel_in_world_[0];
        des_state_traj[i](4) =
            des_state_traj[i - 1](4) + dt_mpc_ * des_body_vel_in_world_[1];
      }
    }
  }

  //=====================================================
  // set contact trajectory
  //=====================================================
  std::vector<ContactState> contact_trajectory;
  for (int i = 0; i < n_horizon_; ++i)
    contact_trajectory.emplace_back(contact_schedule_table[2 * i],
                                    contact_schedule_table[2 * i + 1]);

  //=====================================================
  // set foot pos
  //=====================================================
  aligned_vector<Vector3d> feet_pos_relative_to_body;
  feet_pos_relative_to_body.resize(2);
  Eigen::Vector3d com_pos = robot_->GetRobotComPos();
  com_pos[2] = des_body_height_;
  for (int leg(0); leg < 2; leg++) {
    // feet_pos_relative_to_body[leg] = foot_pos_[leg] - robot_->GetBodyPos();
    feet_pos_relative_to_body[leg] = foot_pos_[leg] - com_pos;
  }

  // set initial state // TODO: check this is correct
  convex_mpc_->setX0(
      des_state_traj[0].head<3>(), des_state_traj[0].segment<3>(3),
      des_state_traj[0].segment<3>(6), des_state_traj[0].tail<3>());
  // set feet pos
  convex_mpc_->setFeetRelativeToBody(feet_pos_relative_to_body);
  // set desired trajectory
  convex_mpc_->setDesiredStateTrajectory(des_state_traj);
  // set contact trajectory
  convex_mpc_->setContactTrajectory(contact_trajectory.data(),
                                    contact_trajectory.size());
  // solve mpc
  convex_mpc_->solve();

  // TODO: get mpc solution
  const auto solution = convex_mpc_->getSolution();
  des_lf_wrench_ = solution.f()[0][0];
  des_rf_wrench_ = solution.f()[0][1];

  // std::cout << "=========================================================="
  //<< std::endl;
  // std::cout << "des_lf_wrench: " << des_lf_wrench_.transpose() << std::endl;
  // std::cout << "des_rf_wrench: " << des_rf_wrench_.transpose() << std::endl;
}

void ConvexMPCLocomotion::_SetupBodyCommand() {
  // filter body velocity command
  double filter = 0.01;
  x_vel_des_ = x_vel_des_ * (1 - filter) + x_vel_cmd_ * filter;
  y_vel_des_ = y_vel_des_ * (1 - filter) + y_vel_cmd_ * filter;
  yaw_rate_des_ = yaw_rate_des_ * (1 - filter) + yaw_rate_cmd_ * filter;

  yaw_des_ = robot_->GetBodyOriYPR()[0] + yaw_rate_des_ * dt_;
  roll_des_ = 0.0;
  pitch_des_ = 0.0;
}

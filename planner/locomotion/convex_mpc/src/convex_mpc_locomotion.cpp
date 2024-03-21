#include "convex_mpc/convex_mpc_locomotion.hpp"
#include "configuration.hpp"
#include "controller/models/composite_rigid_body_inertia.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "util/util.hpp"

#include <cmath>

#include "controller/draco_controller/draco_state_provider.hpp"

ConvexMPCLocomotion::ConvexMPCLocomotion(
    const double dt, const int iterations_btw_mpc, PinocchioRobotSystem *robot,
    bool b_save_mpc_solution, MPCParams *mpc_params,
    CompositeRigidBodyInertia *crbi, void *state_provider)
    : dt_(dt), iterations_btw_mpc_(iterations_btw_mpc), n_horizon_(10),
      robot_(robot), iteration_counter_(0),
      standing_(n_horizon_, Eigen::Vector2i(0, 0), Eigen::Vector2i(10, 10),
                "standing"),
      walking_(n_horizon_, Eigen::Vector2i(0, 5), Eigen::Vector2i(5, 5),
               "walking"),
      gait_for_inertia_(n_horizon_, Eigen::Vector2i(0, 5),
                        Eigen::Vector2i(5, 5), "gait_for_inertia") {
  util::PrettyConstructor(2, "ConvexMPCLocomotion");

  rpy_int_.setZero();
  rpy_comp_.setZero();

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

  b_save_mpc_solution_ = b_save_mpc_solution;

  // convex mpc formulation
  if (mpc_params)
    _InitializeConvexMPC(mpc_params);
  else
    _InitializeConvexMPC();

  // CRBI model
  if (crbi) {
    crbi_ = crbi;
  }

  // state provider (TODO: make it general)
  if (state_provider) {
    sp_ = static_cast<DracoStateProvider *>(state_provider);
  }

#if B_USE_MATLOGGER
  if (b_save_mpc_solution_) {
    logger_ = XBot::MatLogger2::MakeLogger("/tmp/convex_mpc");
    logger_->set_buffer_mode(XBot::VariableBuffer::Mode::producer_consumer);
    appender_ = XBot::MatAppender::MakeInstance();
    appender_->add_logger(logger_);
    appender_->start_flush_thread();
  }
#endif
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
    stand_traj_[0] = 0.0; // roll
    stand_traj_[1] = 0.0; // pitch
    // stand_traj_[0] = sp_->wbo_ypr_[2];           // roll
    // stand_traj_[1] = sp_->wbo_ypr_[1];           // pitch
    stand_traj_[2] = robot_->GetBodyOriYPR()[0]; // yaw
    // stand_traj_[2] = sp_->wbo_ypr_[0]; // yaw
    // stand_traj_[3] = robot_->GetBodyPos()[0];    // TODO:base CoM x
    // stand_traj_[4] = robot_->GetBodyPos()[1];    // TODO:base CoM y
    stand_traj_[3] = robot_->GetRobotComPos()[0];
    stand_traj_[4] = robot_->GetRobotComPos()[1];
    stand_traj_[5] =
        des_body_height_; // des height should be same as stand up height

    des_body_pos_in_world_[0] = stand_traj_[3];
    des_body_pos_in_world_[1] = stand_traj_[4];

    // for inertia computation
    stand_base_traj_[0] = 0.0; // roll
    stand_base_traj_[1] = 0.0; // pitch
    // stand_base_traj_[0] = sp_->wbo_ypr_[2];           // roll
    // stand_base_traj_[1] = sp_->wbo_ypr_[1];           // pitch
    stand_base_traj_[2] = robot_->GetBodyOriYPR()[0]; // yaw
    // stand_base_traj_[2] = sp_->wbo_ypr_[0];            // yaw
    stand_base_traj_.tail<3>() = robot_->GetBodyPos(); // base com x,y,z
    des_base_com_pos_in_world_[0] = stand_base_traj_[3];
    des_base_com_pos_in_world_[1] = stand_base_traj_[4];
  }

  // TEST
  // std::cout << "=================================================="
  //<< std::endl;
  // std::cout << "com xy pos: " <<
  // robot_->GetRobotComPos().head<2>().transpose()
  //<< std::endl;
  // std::cout << "base xy pos: " << robot_->GetBodyPos().head<2>().transpose()
  //<< std::endl;
  // std::cout << "=============================" << std::endl;
  // std::cout << "com vel: " << robot_->GetRobotComLinVel().transpose()
  //<< std::endl;
  // std::cout << "base vel: " << robot_->GetBodyVel().transpose() << std::endl;
  // TEST

  // pick gait
  Gait *gait = &standing_;
  if (gait_number_ == gait::kStand)
    gait = &standing_;
  else if (gait_number_ == gait::kWalking)
    gait = &walking_;
  current_gait_number_ = gait_number_;

  // TODO:integrate position setpoint
  des_body_vel_in_body_ =
      Eigen::Vector3d(x_vel_des_, y_vel_des_, 0.0); // in local body frame
  // des_body_vel_in_world_ = robot_->GetBodyOriRot() * des_body_vel_in_body_;
  des_body_vel_in_world_ =
      yaw_rate_des_ == 0.0
          ? util::SO3FromRPY(0.0, 0.0, stand_traj_[2]) * des_body_vel_in_body_
          : util::SO3FromRPY(0.0, 0.0, robot_->GetBodyOriYPR()[0]) *
                //: util::SO3FromRPY(0.0, 0.0, sp_->wbo_ypr_[0]) *
                des_body_vel_in_body_;
  Eigen::Vector3d body_vel_in_world = robot_->GetRobotComLinVel();
  // body_vel_in_world[2] = robot_->GetBodyVel()[2];

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

  // std::cout << "=================================================" <<
  // std::endl; std::cout << "MPC model" << std::endl; std::cout << "foot pos: "
  //<< "LF: " << foot_pos_[0].transpose()
  //<< "RF: " << foot_pos_[1].transpose() << std::endl;
  // std::cout << "foot ori: "
  //<< "LF: " << Eigen::Quaterniond(foot_ori_[0]).coeffs().transpose()
  //<< "RF: " << Eigen::Quaterniond(foot_ori_[1]).coeffs().transpose()
  //<< std::endl;

  // set body states desired
  if (gait != &standing_) {
    des_body_pos_in_world_ +=
        dt_ * Eigen::Vector3d(des_body_vel_in_world_[0],
                              des_body_vel_in_world_[1], 0.0);
    des_base_com_pos_in_world_ +=
        dt_ * Eigen::Vector3d(des_body_vel_in_world_[0],
                              des_body_vel_in_world_[1], 0.0);
  }

  // first visit initialization
  if (b_first_visit_) {
    // for body pos task
    des_body_pos_in_world_[0] =
        // robot_->GetBodyPos()[0]; // TODO: com xy vs body com xy
        robot_->GetRobotComPos()[0]; // TODO: com xy vs body com xy
    des_body_pos_in_world_[1] =
        // robot_->GetBodyPos()[1]; // TODO: com xy vs body com xy
        robot_->GetRobotComPos()[1]; // TODO: com xy vs body com xy

    des_base_com_pos_in_world_[0] = robot_->GetBodyPos()[0];
    des_base_com_pos_in_world_[1] = robot_->GetBodyPos()[1];

    for (int i = 0; i < 2; ++i) {
      // initialize swing foot trajectory
      foot_swing_trajectory_[i].SetInitialPosition(foot_pos_[i]);
      foot_swing_trajectory_[i].SetFinalPosition(foot_pos_[i]);
      foot_swing_trajectory_[i].SetHeight(swing_height_);

      // initialize swing foot trajectory for inertia traj
      foot_swing_trajectory_for_inertia_[i].SetInitialPosition(foot_pos_[i]);
      foot_swing_trajectory_for_inertia_[i].SetFinalPosition(foot_pos_[i]);
      foot_swing_trajectory_for_inertia_[i].SetHeight(swing_height_);

      // initialize swing foot ori trajectory
      foot_swing_ori_trajectory_[i].SetInitial(Eigen::Quaterniond(foot_ori_[i]),
                                               Eigen::Vector3d::Zero());
      foot_swing_ori_trajectory_[i].SetDesired(Eigen::Quaterniond(foot_ori_[i]),
                                               Eigen::Vector3d::Zero());
      // initialize swing foot ori trajectory for inertia traj
      foot_swing_ori_trajectory_for_inertia_[i].SetInitial(
          Eigen::Quaterniond(foot_ori_[i]), Eigen::Vector3d::Zero());
      foot_swing_ori_trajectory_for_inertia_[i].SetDesired(
          Eigen::Quaterniond(foot_ori_[i]), Eigen::Vector3d::Zero());
    }
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
  double v_abs = std::fabs(des_body_vel_in_body_[0]);
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
        // util::CoordinateRotation(util::CoordinateAxis::Z,
        //-2.0 * yaw_rate_des_ * stance_time) *
        util::CoordinateRotation(util::CoordinateAxis::Z,
                                 -yaw_rate_des_ * stance_time / 2.) *
        foot_pos_from_body;

    Eigen::Vector3d des_vel;
    des_vel[0] = x_vel_des_;
    des_vel[1] = y_vel_des_;
    des_vel[2] = 0.0;

    Eigen::Vector3d des_foot_pos =
        robot_->GetBodyPos() +
        robot_->GetBodyOriRot() *
            (foot_yaw_corrected + des_vel * swing_time_remaining_[leg]);

    // double p_rel_max = 0.4;
    double p_rel_max = 1.0;
    // Using the estimated velocity is correct
    double pfx_rel =
        body_vel_in_world[0] * 0.5 * stance_time +
        // des_body_vel_in_world_[0] * 0.5 * stance_time +
        raibert_gain_ * (body_vel_in_world[0] - des_body_vel_in_world_[0]) +
        //(high_speed_turning_gain_ * robot_->GetBodyPos()[2] / 9.81) *
        (1.0 * robot_->GetRobotComPos()[2] / 9.81) *
            (body_vel_in_world[1] * yaw_rate_des_);
    //(des_body_vel_in_world_[1] * yaw_rate_des_);

    float pfy_rel =
        body_vel_in_world[1] * 0.5 * stance_time +
        // des_body_vel_in_world_[1] * 0.5 * stance_time +
        raibert_gain_ * (body_vel_in_world[1] - des_body_vel_in_world_[1]) +
        //(high_speed_turning_gain_ * robot_->GetBodyPos()[2] / 9.81) *
        (1.0 * robot_->GetRobotComPos()[2] / 9.81) *
            (-body_vel_in_world[0] * yaw_rate_des_);
    //(-des_body_vel_in_world_[0] * yaw_rate_des_);
    pfx_rel = fmin(fmax(pfx_rel, -p_rel_max), p_rel_max);
    pfy_rel = fmin(fmax(pfy_rel, -p_rel_max), p_rel_max);
    des_foot_pos[0] += pfx_rel;
    des_foot_pos[1] += pfy_rel;
    // des_foot_pos[2] = -0.003;
    des_foot_pos[2] = 0.0;
    foot_swing_trajectory_[leg].SetFinalPosition(des_foot_pos);

    // Foot orienation desired design
    // Eigen::Matrix3d world_R_body_yaw =
    // robot_->GetBodyYawRotationMatrix();
    Eigen::Matrix3d world_R_body_yaw =
        yaw_rate_des_ == 0
            ? util::SO3FromRPY(0.0, 0.0, stand_traj_[2])
            : util::SO3FromRPY(0.0, 0.0, robot_->GetBodyOriYPR()[0]);
    //: util::SO3FromRPY(0.0, 0.0, sp_->wbo_ypr_[0]);
    Eigen::Matrix3d des_foot_ori =
        // util::CoordinateRotation(
        // util::CoordinateAxis::Z,
        // yaw_rate_des_ * gait->getStanceDuration(dt_mpc_, foot) / 2.0) *
        // world_R_body_yaw;
        util::CoordinateRotation(util::CoordinateAxis::Z,
                                 yaw_rate_des_ * swing_time_remaining_[leg] /
                                     2.0) *
        world_R_body_yaw;
    foot_swing_ori_trajectory_[leg].SetDesired(
        Eigen::Quaterniond(des_foot_ori).normalized(), Eigen::Vector3d::Zero());

    // swing foot for inertia
    foot_swing_trajectory_for_inertia_[leg].SetFinalPosition(des_foot_pos);
    foot_swing_ori_trajectory_for_inertia_[leg].SetDesired(
        Eigen::Quaterniond(des_foot_ori).normalized(), Eigen::Vector3d::Zero());
  }

  // calculate gait
  gait->setIterations(iterations_btw_mpc_, iteration_counter_);
  // mpc iteration counter
  // iteration_counter_++;

  contact_states_ = gait->getContactState();
  swing_states_ = gait->getSwingState();

  // TODO: consider not doing this!!
  if (b_first_visit_) {
    contact_states_ << 1.0, 1.0;
    swing_states_ << 0.0, 0.0;

    b_first_visit_ = false;
  }

  int *contact_schedule_table = gait->getMPCGait();

  // solve convex mpc with fixed control frequency
  // if (iteration_counter_ % iterations_btw_mpc_ == 0) {
  if (iteration_counter_ % 4 == 0) {
    clock_.Start();
    _SolveConvexMPC(contact_schedule_table);
    clock_.Stop();
  }

  // contact state for state estimator
  Eigen::Vector2d se_contact_state(0.0, 0.0);

  // create foot trajectory
  for (int foot(0); foot < 2; foot++) {
    double contact_state = contact_states_[foot];
    double swing_state = swing_states_[foot];

    if (swing_state > 0) {
      // foot is in swing
      if (b_first_swing_[foot]) {
        b_first_swing_[foot] = false;
        // swing foot pos
        foot_swing_trajectory_[foot].SetInitialPosition(foot_pos_[foot]);
        foot_swing_trajectory_for_inertia_[foot].SetInitialPosition(
            foot_pos_[foot]);

        // swing foot ori
        foot_swing_ori_trajectory_[foot].SetInitial(
            Eigen::Quaterniond(foot_ori_[foot]).normalized(),
            Eigen::Vector3d::Zero());
        foot_swing_ori_trajectory_for_inertia_[foot].SetInitial(
            Eigen::Quaterniond(foot_ori_[foot]).normalized(),
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
  des_body_rpy_[2] = yaw_rate_des_ == 0 ? stand_traj_[2] : yaw_des_;

  // TODO(SH): check this!!
  // this is in body frame and assuming Euler-angle rate matrix is identity
  // (roll, pitch -> 0)
  des_body_ang_vel_[0] = 0.0;
  des_body_ang_vel_[1] = 0.0;
  des_body_ang_vel_[2] = yaw_rate_des_;

  // contact state
  // contact_state_ = gait->getContactState();

  // mpc wrench command linear interpolation
  // des_lf_wrench_ = mpc_lf_wrench_cmd_first_ +
  //(mpc_lf_wrench_cmd_second_ - mpc_lf_wrench_cmd_first_) /
  // iterations_btw_mpc_ *
  //(iteration_counter_ % 5);
  // des_rf_wrench_ = mpc_rf_wrench_cmd_first_ +
  //(mpc_rf_wrench_cmd_second_ - mpc_rf_wrench_cmd_first_) /
  // iterations_btw_mpc_ *
  //(iteration_counter_ % 5);
  des_lf_wrench_ = mpc_lf_wrench_cmd_first_;
  des_rf_wrench_ = mpc_rf_wrench_cmd_first_;

  // mpc iteration counter
  iteration_counter_++;

// TEST
#if B_USE_MATLOGGER
  Eigen::Vector3d actual_cam = robot_->GetHg().head<3>();
  // TODO : this is wrong (should be in torso_com frame not torso link)
  Eigen::Vector3d approx_cam =
      I_global_ *
      robot_->GetLinkSpatialVel(robot_->GetRootFrameName()).head<3>();
  Eigen::Vector3d cam_diff = actual_cam - approx_cam;
  Eigen::Vector3d diff_percent = cam_diff.array() / approx_cam.array();
  // std::cout << "======================================================"
  //<< std::endl;
  // std::cout << "actual cam: " << actual_cam.transpose() << std::endl;
  // std::cout << "approx cam: " << approx_cam.transpose() << std::endl;
  // std::cout << "cam diff percent: " << diff_percent.transpose() << std::endl;
  // std::cout << "Ag matrix: " << robot_->GetAg() << std::endl;

  Eigen::Matrix3d I_base = robot_->GetAg().block<3, 3>(0, 3);
  Eigen::Matrix3d I_com = robot_->GetIg().block<3, 3>(0, 0);
  // std::cout << "======================================" << std::endl;
  // std::cout << "I_base: " << std::endl;
  // std::cout << I_base << std::endl;
  // std::cout << "I_com: " << std::endl;
  // std::cout << I_com << std::endl;

  // logger_->add("act_cam", actual_cam);
  // logger_->add("approx_cam", approx_cam);
  // logger_->add("I_base", I_base.diagonal());
  // logger_->add("I_com", I_com.diagonal());

#endif
  // TEST
}
void ConvexMPCLocomotion::_InitializeConvexMPC(MPCParams *mpc_params) {
  dt_mpc_ = dt_ * iterations_btw_mpc_;

  // cost
  Eigen::MatrixXd Qqq = (mpc_params->Qqq_).asDiagonal();
  Eigen::MatrixXd Qvv = (mpc_params->Qvv_).asDiagonal();
  Eigen::MatrixXd Quu = (mpc_params->Quu_).asDiagonal();
  double decay_rate = mpc_params->decay_rate_;
  Eigen::MatrixXd Qqq_terminal = (mpc_params->Qqq_terminal_).asDiagonal();
  Eigen::MatrixXd Qvv_terminal = (mpc_params->Qvv_terminal_).asDiagonal();

  cost_function_ = std::make_shared<CostFunction>(Qqq, Qvv, Quu, decay_rate,
                                                  Qqq_terminal, Qvv_terminal);

  // state equation
  Eigen::Matrix3d nominal_inertia =
      Eigen::Map<Eigen::Matrix3d>(mpc_params->nominal_inertia_.data(), 3, 3);

  state_equation_ = std::make_shared<StateEquation>(
      dt_mpc_, robot_->GetTotalMass(),
      nominal_inertia); // TODO:nominal inertia need to change for inertia
                        // roll-out

  friction_cone_ = std::make_shared<FrictionCone>(
      mpc_params->mu_, mpc_params->fz_min_, mpc_params->fz_max_,
      mpc_params->foot_half_length_, mpc_params->foot_half_width_);

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

  // design desired state trajectory
  aligned_vector<Vector12d> des_state_traj(n_horizon_ + 1);
  Vector12d des_state_traj_initial = Vector12d::Zero();

  if (current_gait_number_ == gait::kStand) {
    // initial state
    des_state_traj_initial.head<3>() =
        Eigen::Vector3d(roll_des_, pitch_des_, stand_traj_[2]); // rpy
    des_state_traj_initial.segment<3>(3) =
        Eigen::Vector3d(stand_traj_[3], stand_traj_[4], stand_traj_[5]); // xyz

    // desired state trajectory
    fill(des_state_traj.begin(), des_state_traj.end(), des_state_traj_initial);

  } else {
    // when not standing

    // initial state compensation strategy
    // Eigen::Vector3d curr_body_pos_in_world = robot_->GetBodyPos();
    Eigen::Vector3d curr_body_pos_in_world = robot_->GetRobotComPos();
    // curr_body_pos_in_world[2] = robot_->GetBodyPos()[2]; // base height

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
    // des_state_traj_initial.head<3>() = Eigen::Vector3d(
    // rpy_comp_[0], rpy_comp_[1],
    // robot_->GetBodyOriYPR()[0]);
    // des_state_traj_initial.head<3>() =
    // yaw_rate_des_ == 0.0
    //? Eigen::Vector3d(rpy_comp_[0], rpy_comp_[1], stand_traj_[2])
    //: Eigen::Vector3d(rpy_comp_[0], rpy_comp_[1],
    // robot_->GetBodyOriYPR()[0]); // TODO:TEST this
    // (0., 0., yaw_des_)
    des_state_traj_initial.head<3>() =
        yaw_rate_des_ == 0.0
            ? Eigen::Vector3d(0.0, 0.0, stand_traj_[2])
            //? Eigen::Vector3d(stand_traj_[0], stand_traj_[1], stand_traj_[2])
            : Eigen::Vector3d(0.0, 0.0, robot_->GetBodyOriYPR()[0]);
    //: Eigen::Vector3d(0.0, 0.0, yaw_des_);
    //: Eigen::Vector3d(sp_->wbo_ypr_[2], sp_->wbo_ypr_[1],
    // sp_->wbo_ypr_[0]);
    //? Eigen::Vector3d(rpy_comp_[0], rpy_comp_[1], stand_traj_[2])
    //: Eigen::Vector3d(rpy_comp_[0], rpy_comp_[1], sp_->wbo_ypr_[0]);
    //: Eigen::Vector3d(0.0, 0.0, sp_->wbo_ypr_[0]);
    des_state_traj_initial.segment<3>(3) =
        Eigen::Vector3d(x_start, y_start, des_body_height_);

    des_state_traj_initial.segment<3>(6) << 0., 0., yaw_rate_des_;
    des_state_traj_initial.segment<3>(9) = Eigen::Vector3d(
        des_body_vel_in_world_[0], des_body_vel_in_world_[1], 0.0);

    // initial desired state traj
    des_state_traj[0] = des_state_traj_initial;
    des_state_traj[0][5] = des_body_height_;

    for (int i = 1; i < n_horizon_ + 1; ++i) {
      des_state_traj[i][0] = des_state_traj[i - 1][0]; // roll
      des_state_traj[i][1] = des_state_traj[i - 1][1]; // pitch

      if (yaw_rate_des_ == 0.0)
        des_state_traj[i][2] = stand_traj_[2];
      else
        des_state_traj[i][2] =
            des_state_traj[i - 1][2] + dt_mpc_ * yaw_rate_des_;
      // x
      if (des_body_vel_in_world_[0] == 0.0)
        des_state_traj[i][3] = stand_traj_[3];
      else {
        des_state_traj[i][3] =
            // des_state_traj[i - 1][3] + dt_mpc_ * des_body_vel_in_world_[0];
            des_state_traj[i - 1][3] +
            dt_mpc_ *
                util::SO3FromRPY(0.0, 0.0, des_state_traj[i - 1][2])(0, 0) *
                des_body_vel_in_body_[0];
        des_state_traj[i][4] =
            // des_state_traj[i - 1][3] + dt_mpc_ * des_body_vel_in_world_[0];
            des_state_traj[i - 1][4] +
            dt_mpc_ *
                util::SO3FromRPY(0.0, 0.0, des_state_traj[i - 1][2])(1, 0) *
                des_body_vel_in_body_[0];
      }
      // y
      if (des_body_vel_in_world_[1] == 0.0)
        des_state_traj[i][4] = stand_traj_[4];
      // else
      // des_state_traj[i][4] =
      // des_state_traj[i - 1][4] +
      // dt_mpc_ *
      // util::SO3FromRPY(0.0, 0.0, des_state_traj[i - 1][2])(1, 1) *
      // des_body_vel_in_body_[1];

      // z
      des_state_traj[i][5] = des_state_traj[i - 1][5];

      // vel
      des_state_traj[i].tail<6>() = des_state_traj[i - 1].tail<6>();
    }

    // update with current states
    des_state_traj[0][0] = robot_->GetBodyOriYPR()[2];
    des_state_traj[0][1] = robot_->GetBodyOriYPR()[1];
    des_state_traj[0][2] = robot_->GetBodyOriYPR()[0];
    // des_state_traj[0][2] = sp_->wbo_ypr_[0];
    // des_state_traj[0].head<3>() << sp_->wbo_ypr_[2], sp_->wbo_ypr_[1],
    // sp_->wbo_ypr_[0];
    des_state_traj[0][3] = robot_->GetRobotComPos()[0];
    des_state_traj[0][4] = robot_->GetRobotComPos()[1];
    // des_state_traj[0][5] = robot_->GetBodyPos()[2];
    des_state_traj[0][5] = robot_->GetRobotComPos()[2]; // com height
    Eigen::Vector3d torso_ang_vel =
        robot_->GetLinkSpatialVel(robot_->GetRootFrameName()).head<3>();
    des_state_traj[0].segment<3>(6) = torso_ang_vel;
    // des_state_traj[0][8] = sp_->wbo_ang_vel_[2]; // wbo ang vel
    // des_state_traj[0].segment<3>(6) = sp_->wbo_ang_vel_;
    des_state_traj[0].segment<3>(9) = robot_->GetRobotComLinVel();
    // des_state_traj[0][11] = robot_->GetBodyVel()[2];
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
  // TODO: check this quantity
  aligned_vector<Vector3d> feet_pos_relative_to_body;
  feet_pos_relative_to_body.resize(2);
  Eigen::Vector3d com_pos = robot_->GetRobotComPos();
  // Eigen::Vector3d body_pos = robot_->GetBodyPos();
  // com_pos[2] = body_pos[2];
  // com_pos[2] = des_body_height_;
  for (int leg(0); leg < 2; leg++) {
    // feet_pos_relative_to_body[leg] = foot_pos_[leg] - robot_->GetBodyPos();
    feet_pos_relative_to_body[leg] = foot_pos_[leg] - com_pos;
  }

  //=====================================================
  // compute inertia trajectory
  //=====================================================
  // if (crbi_) {
  // input: feet trajectory, base trajectory
  // output: inertia trajectory (alinged_vector< Eigen::Matrix3d>)
  //========================================================
  // desired base trajectory generation
  //========================================================
  aligned_vector<Eigen::Vector3d> des_base_ori_traj(n_horizon_);
  aligned_vector<Eigen::Vector3d> des_base_pos_traj(n_horizon_);

  aligned_vector<Eigen::Vector3d> des_com_pos_traj(n_horizon_);

  des_base_ori_traj[0] =
      yaw_rate_des_ == 0.0
          ? Eigen::Vector3d(0.0, 0.0, stand_base_traj_[2])
          : Eigen::Vector3d(0.0, 0.0, robot_->GetBodyOriYPR()[0]);

  // initial state compensation strategy
  Eigen::Vector3d curr_base_com_pos_in_world = robot_->GetBodyPos();

  const double max_pos_error = 0.1;
  double x_start = des_base_com_pos_in_world_[0];
  double y_start = des_base_com_pos_in_world_[1];

  if (x_start - curr_base_com_pos_in_world[0] > max_pos_error)
    x_start = curr_base_com_pos_in_world[0] + max_pos_error;
  if (curr_base_com_pos_in_world[0] - x_start > max_pos_error)
    x_start = curr_base_com_pos_in_world[0] - max_pos_error;

  if (y_start - curr_base_com_pos_in_world[1] > max_pos_error)
    y_start = curr_base_com_pos_in_world[1] + max_pos_error;
  if (curr_base_com_pos_in_world[1] - y_start > max_pos_error)
    y_start = curr_base_com_pos_in_world[1] - max_pos_error;

  // save des xy pos for wbc
  des_base_com_pos_in_world_[0] = x_start;
  des_base_com_pos_in_world_[1] = y_start;

  des_base_pos_traj[0] =
      Eigen::Vector3d(des_base_com_pos_in_world_[0],
                      des_base_com_pos_in_world_[1], stand_base_traj_[5]);

  // des com x, y & base height
  des_com_pos_traj[0] =
      Eigen::Vector3d(des_body_pos_in_world_[0], des_body_pos_in_world_[1],
                      // stand_base_traj_[5]);
                      stand_traj_[5]);

  for (int i = 1; i < n_horizon_; i++) {
    des_base_ori_traj[i] = des_base_ori_traj[i - 1] +
                           dt_mpc_ * Eigen::Vector3d(0.0, 0.0, yaw_rate_des_);
    // x
    if (des_body_vel_in_world_[0] == 0.0)
      des_base_pos_traj[i][0] = stand_base_traj_[3];
    else {
      des_base_pos_traj[i][0] =
          des_base_pos_traj[i - 1][0] +
          dt_mpc_ *
              util::SO3FromRPY(0.0, 0.0, des_base_ori_traj[i - 1][2])(0, 0) *
              des_body_vel_in_body_[0];
      des_base_pos_traj[i][1] =
          des_base_pos_traj[i - 1][1] +
          dt_mpc_ *
              util::SO3FromRPY(0.0, 0.0, des_base_ori_traj[i - 1][2])(1, 0) *
              des_body_vel_in_body_[0];
    }

    // y
    if (des_body_vel_in_world_[1] == 0.0)
      des_base_pos_traj[i][1] = stand_base_traj_[4];
    // else
    // des_base_pos_traj[i][1] =
    // des_base_pos_traj[i - 1][1] +
    // dt_mpc_ *
    // util::SO3FromRPY(0.0, 0.0, des_base_ori_traj[i - 1][2])(1, 1) *
    // des_body_vel_in_body_[1];
    // z
    des_base_pos_traj[i][2] = des_base_pos_traj[i - 1][2];

    //==================================
    // com ref
    //==================================
    if (des_body_vel_in_world_[0] == 0.0)
      des_com_pos_traj[i][0] = stand_traj_[3];
    else {
      des_com_pos_traj[i][0] =
          des_com_pos_traj[i - 1][0] +
          dt_mpc_ *
              util::SO3FromRPY(0.0, 0.0, des_base_ori_traj[i - 1][2])(0, 0) *
              des_body_vel_in_body_[0];
      des_com_pos_traj[i][1] =
          des_com_pos_traj[i - 1][1] +
          dt_mpc_ *
              util::SO3FromRPY(0.0, 0.0, des_base_ori_traj[i - 1][2])(1, 0) *
              des_body_vel_in_body_[0];
    }
    // y
    if (des_body_vel_in_world_[1] == 0.0)
      des_com_pos_traj[i][1] = stand_traj_[4];
    // else
    // des_com_pos_traj[i][1] =
    // des_com_pos_traj[i - 1][1] +
    // dt_mpc_ *
    // util::SO3FromRPY(0.0, 0.0, des_base_ori_traj[i - 1][2])(1, 1) *
    // des_body_vel_in_body_[1];
    // z
    des_com_pos_traj[i][2] = des_com_pos_traj[i - 1][2];
  }

  //========================================================
  // desired foot trajectory generation
  //========================================================
  // aligned_vector<Eigen::Vector3d> des_lf_ori_traj(n_horizon_);
  // aligned_vector<Eigen::Vector3d> des_lf_pos_traj(n_horizon_);
  // aligned_vector<Eigen::Vector3d> des_rf_ori_traj(n_horizon_);
  // aligned_vector<Eigen::Vector3d> des_rf_pos_traj(n_horizon_);

  std::vector<aligned_vector<Eigen::Vector3d>> des_foot_pos_traj(
      2, aligned_vector<Eigen::Vector3d>(n_horizon_)); // lfoot, rfoot traj
  std::vector<aligned_vector<Eigen::Vector3d>> des_foot_ori_traj(
      2, aligned_vector<Eigen::Vector3d>(n_horizon_)); // lfoot, rfoot traj

  // initial foot pos setup
  for (int foot(0); foot < 2; foot++) {
    des_foot_pos_traj[foot][0] = foot_pos_[foot];
    des_foot_ori_traj[foot][0] = util::RPYFromSO3(foot_ori_[foot]);
  }

  // swing phase iteration
  gait_for_inertia_.setIterations(iterations_btw_mpc_, iteration_counter_);
  Eigen::Vector2d swing_states = gait_for_inertia_.getSwingState();
  Eigen::Vector2d contact_states = gait_for_inertia_.getContactState();
  Eigen::Vector2d swing_states_old = gait_for_inertia_.getSwingState();
  Eigen::Vector2d contact_states_old = gait_for_inertia_.getContactState();

  // first visit
  if (b_first_visit_inertia_gen_) {
    swing_states << 0.0, 0.0;
    swing_states_old << 0.0, 0.0;

    for (int foot = 0; foot < 2; foot++) {
      foot_swing_trajectory_for_inertia_[foot].SetInitialPosition(
          foot_pos_[foot]);
      foot_swing_ori_trajectory_for_inertia_[foot].SetInitial(
          Eigen::Quaterniond(foot_ori_[foot]).normalized(),
          Eigen::Vector3d::Zero());
    }

    b_first_visit_inertia_gen_ = false;
  }

  // std::cout << "===========================================" << std::endl;
  // std::cout << "swing_states: " << swing_states.transpose() << std::endl;

  // iterate over the foot
  for (int foot = 0; foot < 2; foot++) {
    int foot_swing_state_change_count(0);
    Eigen::Vector3d prev_contact_foot_pos = Eigen::Vector3d::Zero();
    Eigen::Quaterniond prev_contact_foot_quat = Eigen::Quaterniond::Identity();
    // iterate over the prediction horizon
    for (int i = 1; i < n_horizon_; i++) {
      // gait phase calculation
      gait_for_inertia_.setIterations(
          iterations_btw_mpc_, iteration_counter_ + iterations_btw_mpc_ * i);
      swing_states = gait_for_inertia_.getSwingState();
      contact_states = gait_for_inertia_.getContactState();
      // std::cout << "swing_states: " << swing_states.transpose() <<
      // std::endl;

      // TODO: assume orienation is constant (forward walking case)
      // des_foot_ori_traj[foot][i] = des_foot_ori_traj[foot][i - 1];

      // foot linear position prediction
      if (swing_states_old[foot] > 0.0 && swing_states[foot] > 0.0) {
        // swing -> swing
        if (foot_swing_state_change_count == 0) {
          foot_swing_trajectory_for_inertia_[foot].ComputeSwingTrajectoryBezier(
              swing_states[foot]);
          des_foot_pos_traj[foot][i] =
              foot_swing_trajectory_for_inertia_[foot].GetPosition();

          // orientation
          des_foot_ori_traj[foot][i] = util::QuatToEulerXYZ(
              foot_swing_ori_trajectory_for_inertia_[foot].GetOrientation(
                  swing_states[foot]));

        } else if (foot_swing_state_change_count == 1) {
          foot_swing_trajectory_for_inertia_[foot].ComputeSwingTrajectoryBezier(
              swing_states[foot]);
          des_foot_pos_traj[foot][i] =
              foot_swing_trajectory_for_inertia_[foot].GetPosition();

          // orientation
          des_foot_ori_traj[foot][i] = util::QuatToEulerXYZ(
              foot_swing_ori_trajectory_for_inertia_[foot].GetOrientation(
                  swing_states[foot]));

        } else if (foot_swing_state_change_count == 2) {
          // TODO:
          // des_foot_pos_traj[foot][i] = des_foot_pos_traj[foot][i - 1];
          des_foot_pos_traj[foot][i] = prev_contact_foot_pos;
          foot_swing_trajectory_for_inertia_[foot].ComputeSwingTrajectoryBezier(
              swing_states[foot]);
          des_foot_pos_traj[foot][i] +=
              foot_swing_trajectory_for_inertia_[foot].GetPosition();
          des_foot_pos_traj[foot][i] -=
              foot_swing_trajectory_for_inertia_[foot].GetInitialPosition();

          // Orientation
          Eigen::Quaterniond delta_quat =
              foot_swing_ori_trajectory_for_inertia_[foot].GetOrientation(
                  swing_states[foot]) *
              foot_swing_ori_trajectory_for_inertia_[foot]
                  .GetInitialOri()
                  .inverse();
          Eigen::Quaterniond final_quat = delta_quat * prev_contact_foot_quat;
          des_foot_ori_traj[foot][i] = util::QuatToEulerXYZ(final_quat);
        }
      } else if (swing_states_old[foot] > 0.0 && swing_states[foot] == 0) {
        // swing -> contact
        foot_swing_state_change_count++;
        des_foot_pos_traj[foot][i] =
            foot_swing_trajectory_for_inertia_[foot].GetFinalPosition();

        // orientation
        des_foot_ori_traj[foot][i] = util::QuatToEulerXYZ(
            foot_swing_ori_trajectory_for_inertia_[foot].GetFinalOri());

      } else if (swing_states_old[foot] == 0.0 && swing_states[foot] > 0.0) {
        // contact -> swing
        foot_swing_state_change_count++;
        if (foot_swing_state_change_count == 1) {
          foot_swing_trajectory_for_inertia_[foot].SetInitialPosition(
              des_foot_pos_traj[foot][i - 1]);
          foot_swing_trajectory_for_inertia_[foot].ComputeSwingTrajectoryBezier(
              swing_states[foot]);
          des_foot_pos_traj[foot][i] =
              foot_swing_trajectory_for_inertia_[foot].GetPosition();

          // orientation
          foot_swing_ori_trajectory_for_inertia_[foot].SetInitial(
              util::EulerZYXtoQuat(des_foot_ori_traj[foot][i - 1]),
              Eigen::Vector3d::Zero());
          des_foot_ori_traj[foot][i] = util::QuatToEulerXYZ(
              foot_swing_ori_trajectory_for_inertia_[foot].GetOrientation(
                  swing_states[foot]));

        } else if (foot_swing_state_change_count == 2) {
          des_foot_pos_traj[foot][i] = des_foot_pos_traj[foot][i - 1];
          prev_contact_foot_pos = des_foot_pos_traj[foot][i - 1];
          foot_swing_trajectory_for_inertia_[foot].ComputeSwingTrajectoryBezier(
              swing_states[foot]);
          des_foot_pos_traj[foot][i] +=
              foot_swing_trajectory_for_inertia_[foot].GetPosition();
          des_foot_pos_traj[foot][i] -=
              foot_swing_trajectory_for_inertia_[foot].GetInitialPosition();

          // orientation
          prev_contact_foot_quat =
              util::EulerZYXtoQuat(des_foot_ori_traj[foot][i - 1]);
          Eigen::Quaterniond delta_quat =
              foot_swing_ori_trajectory_for_inertia_[foot].GetOrientation(
                  swing_states[foot]) *
              foot_swing_ori_trajectory_for_inertia_[foot]
                  .GetInitialOri()
                  .inverse();
          Eigen::Quaterniond final_quat = delta_quat * prev_contact_foot_quat;
          des_foot_ori_traj[foot][i] = util::QuatToEulerXYZ(final_quat);
        }

      } else if (swing_states_old[foot] == 0.0 && swing_states[foot] == 0.0) {
        // contact -> contact
        des_foot_pos_traj[foot][i] = des_foot_pos_traj[foot][i - 1];
        des_foot_ori_traj[foot][i] = des_foot_ori_traj[foot][i - 1];
      }
      // save the previous swing/contact states
      swing_states_old = swing_states;
      contact_states_old = contact_states;

      // make zero on roll and pitch (assume flat terrain)
      des_foot_ori_traj[foot][i](0) = 0.0;
      des_foot_ori_traj[foot][i](1) = 0.0;
    }
  }
  // set inertia prediction
  aligned_vector<Eigen::Matrix3d> crbi_traj(n_horizon_);
  for (int i = 0; i < n_horizon_; ++i) {
    crbi_traj[i] =
        crbi_->ComputeInertia(des_base_pos_traj[i], des_base_ori_traj[i],
                              des_foot_pos_traj[0][i], des_foot_ori_traj[0][i],
                              des_foot_pos_traj[1][i], des_foot_ori_traj[1][i]);
  }
  convex_mpc_->setInertiaTrajectory(crbi_traj);
  // }

  // TODO: baseline methods-->>set body inertia (update every mpc loop)
  Eigen::Matrix3d I_global = robot_->GetIg().topLeftCorner<3, 3>();
  I_global_ = I_global;
  Eigen::Matrix3d global_R_local = robot_->GetBodyOriRot();
  Eigen::Matrix3d I_local =
      global_R_local.transpose() * I_global * global_R_local;
  convex_mpc_->setBodyInertia(I_local);

  // foot pos relative body trajectory
  std::vector<aligned_vector<Eigen::Vector3d>> feet_pos_relative_to_body_traj(
      2, aligned_vector<Eigen::Vector3d>(n_horizon_));
  for (int foot = 0; foot < 2; ++foot) {
    for (int i = 0; i < n_horizon_; ++i) {
      feet_pos_relative_to_body_traj[foot][i] =
          des_foot_pos_traj[foot][i] - des_com_pos_traj[i];
    }
  }
  // set initial state // TODO: check this is correct
  convex_mpc_->setX0(
      des_state_traj[0].head<3>(), des_state_traj[0].segment<3>(3),
      des_state_traj[0].segment<3>(6), des_state_traj[0].tail<3>());
  // TODO: set feet pos (check this!!!!!) should be trajectory
  convex_mpc_->setFeetRelativeToBody(feet_pos_relative_to_body);
  convex_mpc_->setFeetRelativeToBodyTrajectory(feet_pos_relative_to_body_traj);
  // set desired trajectory
  convex_mpc_->setDesiredStateTrajectory(des_state_traj);
  // set contact trajectory
  convex_mpc_->setContactTrajectory(contact_trajectory.data(),
                                    contact_trajectory.size());
  // solve mpc
  convex_mpc_->solve();

  // TODO: get mpc solution
  const auto mpc_solution = convex_mpc_->getSolution();
  // des_lf_wrench_ = mpc_solution.f()[0][0];
  // des_rf_wrench_ = mpc_solution.f()[0][1];

  mpc_lf_wrench_cmd_first_ = mpc_solution.f()[0][0];
  mpc_lf_wrench_cmd_second_ = mpc_solution.f()[1][0];
  mpc_rf_wrench_cmd_first_ = mpc_solution.f()[0][1];
  mpc_rf_wrench_cmd_second_ = mpc_solution.f()[1][1];

  if (b_save_mpc_solution_) {
    // save mpc solution data for prediction horizon
    logger_->add("time", mpc_solution.time());
    for (const auto &pos : mpc_solution.pos())
      logger_->add("com_pos", pos);
    for (const auto &euler_angle : mpc_solution.euler())
      logger_->add("euler_ang", euler_angle);
    for (const auto &com_vel : mpc_solution.v())
      logger_->add("com_vel", com_vel);
    for (const auto &ang_vel : mpc_solution.w())
      logger_->add("ang_vel", ang_vel);
    for (const auto &contact_wrench : mpc_solution.f()) {
      logger_->add("force_LF", contact_wrench[0]);
      logger_->add("force_RF", contact_wrench[1]);
    }

    // save desired state trajectory provided to the MPC
    for (const auto &des_st : des_state_traj) {
      logger_->add("des_euler_ang", des_st.segment<3>(0));
      logger_->add("des_com_pos", des_st.segment<3>(3));
      logger_->add("des_ang_vel", des_st.segment<3>(6));
      logger_->add("des_com_vel", des_st.segment<3>(9));
    }

    // save desired foot trajectory over the prediction horizon
    for (const auto &des_lf_pos : des_foot_pos_traj[0])
      logger_->add("des_lf_pos", des_lf_pos);
    for (const auto &des_rf_pos : des_foot_pos_traj[1])
      logger_->add("des_rf_pos", des_rf_pos);
    for (const auto &des_lf_ori : des_foot_ori_traj[0])
      logger_->add("des_lf_ori", des_lf_ori);
    for (const auto &des_rf_ori : des_foot_ori_traj[1])
      logger_->add("des_rf_ori", des_rf_ori);
    for (const auto &des_base_pos : des_base_pos_traj)
      logger_->add("des_base_pos", des_base_pos);
    for (const auto &des_base_ori : des_base_ori_traj)
      logger_->add("des_base_ori", des_base_ori);

    // save mpc solve time
    logger_->add("mpc_solve_time", clock_.duration());
  }

  // std::cout <<
  // "=========================================================="
  //<< std::endl;
  // std::cout << "des_lf_wrench: " << des_lf_wrench_.transpose() <<
  // std::endl; std::cout << "des_rf_wrench: " << des_rf_wrench_.transpose()
  // << std::endl;
}

void ConvexMPCLocomotion::_SetupBodyCommand() {
  // filter body velocity command
  double filter = 0.1;
  x_vel_des_ = x_vel_des_ * (1 - filter) + x_vel_cmd_ * filter;
  y_vel_des_ = y_vel_des_ * (1 - filter) + y_vel_cmd_ * filter;
  yaw_rate_des_ = yaw_rate_des_ * (1 - filter) + yaw_rate_cmd_ * filter;

  yaw_des_ = robot_->GetBodyOriYPR()[0] + yaw_rate_des_ * dt_;
  // yaw_des_ = sp_->wbo_ypr_[0] + yaw_rate_des_ * dt_;
  roll_des_ = 0.0;
  pitch_des_ = 0.0;
}

//===========================================================
// should be deprecated
//===========================================================
void ConvexMPCLocomotion::_InitializeConvexMPC() {
  dt_mpc_ = dt_ * iterations_btw_mpc_;

  // cost
  Eigen::MatrixXd Qqq = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Qvv = Eigen::MatrixXd::Zero(6, 6);
  Eigen::MatrixXd Quu = Eigen::MatrixXd::Zero(6, 6);

  Qqq(0, 0) = 100;  // roll
  Qqq(1, 1) = 100;  // pitch
  Qqq(2, 2) = 150;  // yaw
  Qqq(3, 3) = 200;  // x
  Qqq(4, 4) = 200;  // y
  Qqq(5, 5) = 1000; // z

  Qvv(0, 0) = 1;   // wx
  Qvv(1, 1) = 1;   // wy
  Qvv(2, 2) = 1;   // wz
  Qvv(3, 3) = 1;   // xdot
  Qvv(4, 4) = 1;   // ydot
  Qvv(5, 5) = 100; // zdot

  Quu(0, 0) = 1e-3;
  Quu(1, 1) = 1e-3;
  Quu(2, 2) = 1e-4;
  Quu(3, 3) = 1e-5;
  Quu(4, 4) = 1e-5;
  Quu(5, 5) = 1e-5;

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
  double fz_max = 600.0;
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

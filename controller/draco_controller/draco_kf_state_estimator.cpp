#include "draco_kf_state_estimator.hpp"

#if B_USE_ZMQ
#include "draco_data_manager.hpp"
#endif

DracoKFStateEstimator::DracoKFStateEstimator(PinocchioRobotSystem *_robot) {
  util::PrettyConstructor(1, "DracoKFStateEstimator");
  robot_ = _robot;
  sp_ = DracoStateProvider::GetStateProvider();

  iso_imu_to_base_com_ =
          robot_->GetLinkIsometry(draco_link::torso_imu).inverse() *
          robot_->GetLinkIsometry(draco_link::torso_com_link);
  Eigen::Vector3d rpy = util::RPYFromSO3(iso_imu_to_base_com_.linear().transpose());
  quat_imu_to_base_com_ = util::EulerZYXtoQuat(rpy(0), rpy(1), rpy(2));

  try{
    YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

    bool b_sim = util::ReadParameter<bool>(cfg, "b_sim");
    std::string prefix = b_sim ? "sim" : "exp";

    Eigen::Vector3d sigma_base_vel = util::ReadParameter<Eigen::Vector3d>(
            cfg["state_estimator"], prefix + "_sigma_base_vel");
    Eigen::Vector3d sigma_base_acc = util::ReadParameter<Eigen::Vector3d>(
            cfg["state_estimator"], prefix + "_sigma_base_acc");
    Eigen::Vector3d sigma_pos_lfoot = util::ReadParameter<Eigen::Vector3d>(
            cfg["state_estimator"], prefix + "_sigma_pos_lfoot");
    Eigen::Vector3d sigma_pos_rfoot = util::ReadParameter<Eigen::Vector3d>(
            cfg["state_estimator"], prefix + "_sigma_pos_rfoot");
    Eigen::Vector3d sigma_vel_lfoot = util::ReadParameter<Eigen::Vector3d>(
            cfg["state_estimator"], prefix + "_sigma_vel_lfoot");
    Eigen::Vector3d sigma_vel_rfoot = util::ReadParameter<Eigen::Vector3d>(
            cfg["state_estimator"], prefix + "_sigma_vel_rfoot");
    Eigen::Vector3d imu_accel_bias = util::ReadParameter<Eigen::Vector3d>(
            cfg["state_estimator"], prefix + "_imu_accel_bias");
    Eigen::VectorXd n_data_com_vel = util::ReadParameter<Eigen::VectorXd>(
            cfg["state_estimator"], prefix + "_num_data_com_vel");
    //  Eigen::VectorXd n_data_cam = util::ReadParameter<Eigen::VectorXd>(
    //      cfg["state_estimator"], prefix + "n_data_cam");
    Eigen::VectorXd n_data_base_accel = util::ReadParameter<Eigen::VectorXd>(
            cfg["state_estimator"], prefix + "_num_data_base_accel");
    Eigen::VectorXd n_data_ang_vel = util::ReadParameter<Eigen::VectorXd>(
            cfg["state_estimator"], prefix + "_num_data_ang_vel");
    double time_constant_base_accel = util::ReadParameter<double>(
            cfg["state_estimator"], prefix + "_base_accel_time_constant");
    double time_constant_ang_vel = util::ReadParameter<double>(
            cfg["state_estimator"], prefix + "_ang_vel_time_constant");
    double cut_off_period = util::ReadParameter<double>(
            cfg["state_estimator"], "cut_off_period");
    int foot_frame = util::ReadParameter<int>(
            cfg["state_estimator"], "foot_reference_frame");
    b_use_marg_filter = util::ReadParameter<bool>(cfg["state_estimator"],
            "b_use_marg_filter");
    // Eigen::VectorXd base_accel_limits = util::ReadParameter<Eigen::VectorXd>(
    // cfg["state_estimator"], prefix + "_base_accel_limits");

    // set the foot that will be used to estimate base in first_visit
    if (foot_frame == 0) {
      est_ref_foot_frame_ = draco_link::l_foot_contact;
      est_non_ref_foot_frame_ = draco_link::r_foot_contact;
    } else {
      est_ref_foot_frame_ = draco_link::r_foot_contact;
      est_non_ref_foot_frame_ = draco_link::l_foot_contact;
    }

    for (int i = 0; i < 3; ++i) {
      //    cam_filter_.push_back(SimpleMovingAverage(n_data_cam[i]));
      base_accel_filter_.push_back(SimpleMovingAverage(n_data_base_accel[i]));
      imu_ang_vel_filter_.push_back(SimpleMovingAverage(n_data_ang_vel[i]));
    }
    // Filtered base velocity
    // base_accel_filt_ = new ExponentialMovingAverageFilter(
    // sp_->servo_dt_, time_constant_base_accel, Eigen::VectorXd::Zero(3),
    //-base_accel_limits, base_accel_limits);
    base_accel_filt_ = new FirstOrderLowPassFilter(sp_->servo_dt_,
                                                   time_constant_base_accel, 3);
    imu_ang_vel_filt_ = new FirstOrderLowPassFilter(sp_->servo_dt_,
                                                    time_constant_ang_vel, 3);
    com_vel_filt_ = new LowPassVelocityFilter(sp_->servo_dt_, cut_off_period, 3);

    system_model_.initialize(deltat, sigma_base_vel, sigma_base_acc,
                             sigma_vel_lfoot, sigma_vel_rfoot);
    base_pose_model_.initialize(sigma_pos_lfoot, sigma_pos_rfoot, imu_accel_bias);

    // set-up MARG filter for robot orientation estimation
    if (b_use_marg_filter) {
      margFilter_ = MARGFilter();
    }
  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }

  contact_manager_ = std::make_unique<ContactDetectionManager>(robot_);

  base_acceleration_.setZero();
  x_hat_.setZero();
  rot_world_to_base.setZero();
  global_linear_offset_.setZero();
  prev_base_com_pos_.setZero();
  current_support_state_ = DOUBLE;
  prev_support_state_ = DOUBLE;
  foot_pos_from_base_pre_transition.setZero();
  foot_pos_from_base_post_transition(0) = NAN;
  grav_vec_3D_ << Eigen::Vector3d(0., 0., 9.81);

  b_first_visit_ = true;
  b_skip_prediction = false;
  b_request_offset_reset = false;

#if B_USE_MATLOGGER
  logger_ = XBot::MatLogger2::MakeLogger("/tmp/draco_state_estimator_kf_data");
#endif
}

DracoKFStateEstimator::~DracoKFStateEstimator() {
  delete base_accel_filt_;
  delete imu_ang_vel_filt_;
  delete com_vel_filt_;
}

void DracoKFStateEstimator::Initialize(DracoSensorData *sensor_data) {
  // filter imu angular velocity
  for (int i = 0; i < 3; ++i) {
    imu_ang_vel_filter_[i].Input(sensor_data->imu_ang_vel_[i]);
    //    sp_->imu_ang_vel_est_[i] = imu_ang_vel_filter_[i].Output();
  }
  imu_ang_vel_filt_->Input(sensor_data->imu_ang_vel_);

  // estimate 0_R_b
  Eigen::Quaterniond imu_quat(
      sensor_data->imu_frame_quat_[3], sensor_data->imu_frame_quat_[0],
      sensor_data->imu_frame_quat_[1], sensor_data->imu_frame_quat_[2]);
  Eigen::Matrix3d rot_world_to_imu = imu_quat.normalized().toRotationMatrix();

  if (b_use_marg_filter){
    // orientation estimation
    Eigen::Quaterniond torso_quat;
    torso_quat = rot_world_to_imu * quat_imu_to_base_com_;
    margFilter_.initialize(torso_quat.w(), torso_quat.x(),
                           torso_quat.y(), torso_quat.z());
  }
  rot_world_to_base = compute_world_to_base_rot(sensor_data, rot_world_to_imu,
                                                b_use_marg_filter);

  // compute estimator (control) input, u_n
  //  for (int i = 0; i < 3; ++i) {
  //    base_accel_filter_[i].Input(sensor_data->imu_dvel[i] / sp_->servo_dt_);
  //    base_acceleration_[i] = base_accel_filter_[i].Output();
  //  }
  // base_accel_filt_->Input(sensor_data->imu_dvel_ / sp_->servo_dt_);
  base_accel_filt_->Input(sensor_data->imu_lin_acc_);
  base_acceleration_ = base_accel_filt_->Output();

  base_pose_model_.packAccelerationInput(rot_world_to_imu, base_acceleration_,
                                         accelerometer_input_);

  // update contact filter data
  Eigen::Vector2d contact_normal = Eigen::Vector2d::Zero();
  contact_normal << sensor_data->lf_contact_normal_, sensor_data->rf_contact_normal_;
  contact_manager_->UpdateForceMeasurements(contact_normal);

  // update system without base linear states
  robot_->UpdateRobotModel(
      Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), sensor_data->joint_pos_,
      sensor_data->joint_vel_, false);

  // save data
  if (sp_->count_ % sp_->data_save_freq_ == 0) {
#if B_USE_ZMQ
    DracoDataManager *dm = DracoDataManager::GetDataManager();
    dm->data_->joint_positions_ = sensor_data->joint_pos_;
#endif

#if B_USE_MATLOGGER
    // joint encoder data
    logger_->add("joint_pos_act", sensor_data->joint_pos_);
    logger_->add("joint_vel_act", sensor_data->joint_vel_);
#endif
  }
}

void DracoKFStateEstimator::Update(DracoSensorData *sensor_data) {

  // filter imu angular velocity
  for (int i = 0; i < 3; ++i) {
    imu_ang_vel_filter_[i].Input(sensor_data->imu_ang_vel_[i]);
    //    sp_->imu_ang_vel_est_[i] = imu_ang_vel_filter_[i].Output();
  }
  imu_ang_vel_filt_->Input(sensor_data->imu_ang_vel_);

  // estimate 0_R_b
  Eigen::Quaterniond imu_quat(
      sensor_data->imu_frame_quat_[3], sensor_data->imu_frame_quat_[0],
      sensor_data->imu_frame_quat_[1], sensor_data->imu_frame_quat_[2]);
  Eigen::Matrix3d rot_world_to_imu = imu_quat.normalized().toRotationMatrix();
  if ((b_first_visit_) && (b_use_marg_filter)) {
    // orientation estimation
    Eigen::Quaterniond torso_quat;
    torso_quat = rot_world_to_imu * quat_imu_to_base_com_;
    margFilter_.initialize(torso_quat.w(), torso_quat.x(),
                           torso_quat.y(), torso_quat.z());
  }
  rot_world_to_base = compute_world_to_base_rot(sensor_data, rot_world_to_imu,
                                                b_use_marg_filter);

  // compute estimator (control) input, u_n
  base_accel_filt_->Input(sensor_data->imu_lin_acc_);
  base_acceleration_ = base_accel_filt_->Output();

  base_pose_model_.packAccelerationInput(rot_world_to_imu, base_acceleration_,
                                         accelerometer_input_);

  // update contact filter data
  Eigen::Vector2d contact_normal = Eigen::Vector2d::Zero();
  contact_normal << sensor_data->lf_contact_normal_, sensor_data->rf_contact_normal_;
  contact_manager_->UpdateForceMeasurements(contact_normal);

  // update system without base linear states
  robot_->UpdateRobotModel(Eigen::Vector3d::Zero(),
                           Eigen::Quaterniond(rot_world_to_base).normalized(),
                           Eigen::Vector3d::Zero(), sensor_data->imu_ang_vel_,
                           sensor_data->joint_pos_, sensor_data->joint_vel_,
                           false);

  // get initial base estimate
  Eigen::Vector3d world_to_base;
  // initialize Kalman filter state xhat =[0_pos_b, 0_vel_b, 0_pos_LF, 0_pos_RF]
  if (b_first_visit_) {
    world_to_base =
        global_linear_offset_ -
        robot_->GetLinkIsometry(est_ref_foot_frame_).translation();

    x_hat_.initialize(world_to_base,
                      robot_->GetLinkIsometry(est_ref_foot_frame_),
                      robot_->GetLinkIsometry(est_non_ref_foot_frame_));
    kalman_filter_.init(x_hat_);
  } else {
    world_to_base << x_hat_.base_pos_x(), x_hat_.base_pos_y(),
        x_hat_.base_pos_z();
  }

  // pass expected floor height to contact manager
  // TODO: pass height from vision
  Eigen::Vector2d expected_contact_height = Eigen::Vector2d::Zero();

  // update contact
  //  if (sensor_data->b_rf_contact) {
  //    sp_->b_rf_contact = true;
  //  } else {
  //    sp_->b_rf_contact = false;
  //  }
  //
  //  if (sensor_data->b_lf_contact) {
  //    sp_->b_lf_contact = true;
  //  } else {
  //    sp_->b_lf_contact = false;
  //  }
  contact_manager_->UpdateContactStates(sp_, expected_contact_height);
  updateSupportState(sp_, current_support_state_);

  // at support state change, update global offset and covariance gains
  if (current_support_state_ != prev_support_state_) {

    // from double to right support and viceversa
    if ((prev_support_state_ == DOUBLE) && (current_support_state_ == RIGHT)) {
      foot_pos_from_base_pre_transition =
          x_hat_.tail(6).head(3); // estimated lfoot before lift-off
      base_pose_model_.update_leg_covariance(PoseMeasurementModel::LEFT,
                                             COV_LEVEL_HIGH);
    } else if ((prev_support_state_ == RIGHT) &&
               (current_support_state_ == DOUBLE)) {
      foot_pos_from_base_post_transition =
          x_hat_.head(3) +
          robot_->GetLinkIsometry(draco_link::l_foot_contact).translation();
      global_linear_offset_ = foot_pos_from_base_post_transition -
                              foot_pos_from_base_pre_transition;
      global_linear_offset_.z() =
          0.0; // TODO make more robust later for non-flat ground
      system_model_.update_lfoot_offset(global_linear_offset_);
      b_request_offset_reset = true;
      foot_pos_from_base_post_transition.z() =
          0.0; // TODO make more robust later for non-flat ground
      //      system_model_.update_lfoot_offset(foot_pos_from_base_post_transition);
      //      base_pose_model_.update_lfoot_offset(foot_pos_from_base_post_transition);
      base_pose_model_.update_leg_covariance(PoseMeasurementModel::LEFT,
                                             COV_LEVEL_LOW);
    } else if ((prev_support_state_ == DOUBLE) &&
               (current_support_state_ == LEFT)) {
      // from double support to left support and viceversa
      foot_pos_from_base_pre_transition =
          x_hat_.tail(3); // estimated rfoot before lift-off
      base_pose_model_.update_leg_covariance(PoseMeasurementModel::RIGHT,
                                             COV_LEVEL_HIGH);
    } else if ((prev_support_state_ == LEFT) &&
               (current_support_state_ == DOUBLE)) {
      foot_pos_from_base_post_transition =
          x_hat_.head(3) +
          robot_->GetLinkIsometry(draco_link::r_foot_contact).translation();
      global_linear_offset_ = foot_pos_from_base_post_transition -
                              foot_pos_from_base_pre_transition;
      global_linear_offset_.z() =
          0.0; // TODO make more robust later for non-flat ground
      system_model_.update_rfoot_offset(global_linear_offset_);
      b_request_offset_reset = true;
      foot_pos_from_base_post_transition.z() =
          0.0; // TODO make more robust later for non-flat ground
      //      system_model_.update_rfoot_offset(foot_pos_from_base_post_transition);
      base_pose_model_.update_leg_covariance(PoseMeasurementModel::RIGHT,
                                             COV_LEVEL_LOW);
    }
    if (foot_pos_from_base_post_transition.hasNaN()) {
      x_hat_ = kalman_filter_.predict(system_model_, accelerometer_input_);
      global_linear_offset_.setZero();
      system_model_.update_lfoot_offset(global_linear_offset_);
      system_model_.update_rfoot_offset(global_linear_offset_);
      b_skip_prediction = true;
    }
  }
  if (!b_skip_prediction) {
    x_hat_ = kalman_filter_.predict(system_model_, accelerometer_input_);
    if (b_request_offset_reset) {
      system_model_.reset_offsets();
      b_request_offset_reset = false;
    }
  } else {
    b_skip_prediction = false;
  }

  // update measurement assuming at least one foot is on the ground
  if (sp_->b_lf_contact_) {
    Eigen::Vector3d pos_base_from_lfoot =
        -robot_->GetLinkIsometry(draco_link::l_foot_contact).translation();
    base_pose_model_.update_position_from_lfoot(pos_base_from_lfoot,
                                                base_estimate_);
  }
  if (sp_->b_rf_contact_) {
    Eigen::Vector3d pos_base_from_rfoot =
        -robot_->GetLinkIsometry(draco_link::r_foot_contact).translation();
    base_pose_model_.update_position_from_rfoot(pos_base_from_rfoot,
                                                base_estimate_);
  }
  x_hat_ = kalman_filter_.update(base_pose_model_, base_estimate_);

  // values computed by linear KF estimator
  Eigen::Vector3d base_position_estimate(
      x_hat_.base_pos_x(), x_hat_.base_pos_y(), x_hat_.base_pos_z());
  Eigen::Vector3d base_velocity_estimate(
      x_hat_.base_vel_x(), x_hat_.base_vel_y(), x_hat_.base_vel_z());


  // Update robot model with full estimate
  robot_->UpdateRobotModel(base_position_estimate,
                           Eigen::Quaterniond(rot_world_to_base).normalized(),
                           base_velocity_estimate, imu_ang_vel_filt_->Output(),
                           sensor_data->joint_pos_, sensor_data->joint_vel_,
                           true);

  // filter com velocity, cam
  Eigen::Vector3d com_pos;
  com_pos << robot_->GetRobotComPos();
  if(b_first_visit_) {
    com_vel_filt_->Reset(com_pos);
    b_first_visit_ = false;
  }
  com_vel_filt_->Input(com_pos);
  sp_->com_vel_est_ = com_vel_filt_->Output();

  this->ComputeDCM();

  prev_support_state_ = current_support_state_;
  // save current time step sensor_data
  if (sp_->count_ % sp_->data_save_freq_ == 0) {
#if B_USE_ZMQ
    DracoDataManager *dm = DracoDataManager::GetDataManager();
    dm->data_->joint_positions_ = sensor_data->joint_pos_;
    dm->data_->kf_base_joint_pos_ = base_position_estimate;
    Eigen::Quaterniond quat = Eigen::Quaterniond(rot_world_to_base);
    dm->data_->kf_base_joint_ori_ << quat.x(), quat.y(), quat.z(), quat.w();

    dm->data_->est_icp = sp_->dcm_.head<2>();

    dm->data_->b_lfoot_ = sp_->b_lf_contact_;
    dm->data_->b_rfoot_ = sp_->b_rf_contact_;
    dm->data_->lfoot_volt_normal_raw_ = sensor_data->lf_contact_normal_;
    dm->data_->rfoot_volt_normal_raw_ = sensor_data->rf_contact_normal_;
    dm->data_->lfoot_rf_normal_ = contact_manager_->GetLFootNormalForceRaw();
    dm->data_->rfoot_rf_normal_ = contact_manager_->GetRFootNormalForceRaw();
    dm->data_->lfoot_rf_normal_filt_ = contact_manager_->GetFootNormalForceFilt(end_effector::LFoot);
    dm->data_->rfoot_rf_normal_filt_ = contact_manager_->GetFootNormalForceFilt(end_effector::RFoot);
#endif

#if B_USE_MATLOGGER

    // joint encoder data
    logger_->add("joint_pos_act", sensor_data->joint_pos_);
    logger_->add("joint_vel_act", sensor_data->joint_vel_);

    // floating base estimate data
    logger_->add("base_joint_pos_kf", base_position_estimate);
    logger_->add("base_joint_rpy_kf", util::RPYFromSO3(rot_world_to_base));
    Eigen::Quaterniond base_quat = Eigen::Quaterniond(
            sensor_data->imu_frame_quat_[3], sensor_data->imu_frame_quat_[0],
            sensor_data->imu_frame_quat_[1], sensor_data->imu_frame_quat_[2]);
//    logger_->add("base_joint_rpy_raw", base_quat.toRotationMatrix().eulerAngles(2, 1, 0));
    logger_->add("base_joint_ypr_raw", util::QuatToEulerZYX(base_quat));
    logger_->add("base_joint_rpy_error", margFilter_.getGyroscopeError());
    logger_->add("base_joint_lin_vel_kf", base_velocity_estimate);
    logger_->add("base_joint_ang_vel_kf", imu_ang_vel_filt_->Output());
    //    logger_->add("base_joint_ang_vel_kf", imu_ang_vel_filter_.Output());

    // com velocities
    logger_->add("com_vel_raw", robot_->GetRobotComLinVel());
    logger_->add("com_vel_est", sp_->com_vel_est_);

    // icp data
    logger_->add("icp_est", sp_->dcm_.head<2>());
    logger_->add("icp_vel_est", sp_->dcm_vel_.head<2>());

    // imu accel data
    logger_->add("imu_accel_raw", sensor_data->imu_dvel_ / sp_->servo_dt_);
    logger_->add("world_LA_imu", sensor_data->imu_lin_acc_);
    logger_->add("imu_accel_est", base_acceleration_);

    // save feet contact information (leave for when we integrate contact
    // sensor)
    logger_->add("b_lf_contact_touchdown", contact_manager_->HasContactSensorTouchdown(end_effector::LFoot));
    logger_->add("b_rf_contact_touchdown", contact_manager_->HasContactSensorTouchdown(end_effector::RFoot));
    logger_->add("b_lf_heel_toe_touchdown", contact_manager_->HasHeelToeTouchdown(end_effector::LFoot));
    logger_->add("b_rf_heel_toe_touchdown", contact_manager_->HasHeelToeTouchdown(end_effector::RFoot));
    logger_->add("b_rf_contact", sp_->b_rf_contact_);
    logger_->add("b_lf_contact", sp_->b_lf_contact_);
    logger_->add("act_rf_z_lfoot", contact_manager_->GetFootNormalForceFilt(end_effector::LFoot));
    logger_->add("act_rf_z_rfoot", contact_manager_->GetFootNormalForceFilt(end_effector::RFoot));
//    dm->data_->lfoot_contact_ = sp_->stance_foot_ == "l_foot_contact";
//    dm->data_->rfoot_contact_ = sp_->stance_foot_ == "r_foot_contact";
//    dm->data_->lf_contact_ = sp_->b_lf_contact_;
//    dm->data_->rf_contact_ = sp_->b_rf_contact_;
//
#endif
  }
}

void DracoKFStateEstimator::updateSupportState(DracoStateProvider *sp,
                                               SupportState &support_state) {
  if (sp->b_rf_contact_ && sp->b_lf_contact_) {
    support_state = DOUBLE;
    return;
  }

  if (sp->b_lf_contact_) {
    support_state = LEFT;
    return;
  }

  support_state = RIGHT;
}

Eigen::Matrix3d DracoKFStateEstimator::compute_world_to_base_rot(
    DracoSensorData *data, Eigen::Matrix3d& rot_world_to_imu,
    bool use_marg_filter) {
  if (use_marg_filter) {
    Eigen::Vector3d se_ang_vel, se_lin_acc;
    se_ang_vel = margFilter_.getBaseRotation().transpose() * imu_ang_vel_filt_->Output();
    se_lin_acc = margFilter_.getBaseRotation().transpose() * (base_acceleration_ + grav_vec_3D_);

    margFilter_.filterUpdate(se_ang_vel[0], se_ang_vel[1],
                             se_ang_vel[2], se_lin_acc[0], se_lin_acc[1],
                             se_lin_acc[2]);
    return margFilter_.getBaseRotation();
  } else {
    return rot_world_to_imu * iso_imu_to_base_com_.linear();
  }
}

void DracoKFStateEstimator::ComputeDCM() {
  Eigen::Vector3d com_pos = robot_->GetRobotComPos();
  Eigen::Vector3d com_vel = sp_->com_vel_est_;
  double dcm_omega = sqrt(grav_vec_3D_.z() / com_pos[2]);

  sp_->prev_dcm_ = sp_->dcm_;
  sp_->dcm_ = com_pos + com_vel / dcm_omega;

  double alpha_vel = 0.1; // TODO Study this alpha value
  sp_->dcm_vel_ = alpha_vel * ((sp_->dcm_ - sp_->prev_dcm_) / sp_->servo_dt_) +
                  (1.0 - alpha_vel) * sp_->dcm_vel_;
}

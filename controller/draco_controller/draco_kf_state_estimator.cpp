#include "draco_kf_state_estimator.hpp"

DracoKFStateEstimator::DracoKFStateEstimator(PinocchioRobotSystem *_robot) {
  util::PrettyConstructor(1, "DracoKFStateEstimator");
  robot_ = _robot;
  sp_ = DracoStateProvider::GetStateProvider();

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

  iso_imu_to_base_com_ = robot_->GetLinkIsometry(draco_link::torso_imu).inverse() *
                        robot_->GetLinkIsometry(draco_link::torso_link);

  x_hat_.setZero();
  system_model_.initialize(deltat);
  double gravity = 9.81; //TODO get from somewhere else
  base_pose_model_.initialize(gravity);
  rot_world_to_base.setZero();
  global_linear_offset_.setZero();
  current_support_state_ = DOUBLE;
  prev_support_state_ = DOUBLE;
  foot_pos_from_base_pre_transition.setZero();
  foot_pos_from_base_post_transition(0) = NAN;

  //TODO move settings to config/draco/pnc.yaml
  b_first_visit_ = true;
  b_skip_prediction = false;
  b_use_marg_filter = false;
}

DracoKFStateEstimator::~DracoKFStateEstimator() {
}

void DracoKFStateEstimator::initialize(DracoSensorData *data) {
    this->update(data);
}

void DracoKFStateEstimator::update(DracoSensorData *data) {

  // continue only when state estimator is on
//   if (!(sp_->isStateEstimatorOn()))
//    return;

  // estimate 0_R_b
  Eigen::Quaternion<double> imu_frame_quat(data->imu_frame_quat_(3), data->imu_frame_quat_(0), data->imu_frame_quat_(1), data->imu_frame_quat_(2));
  Eigen::Matrix3d rot_world_to_imu = imu_frame_quat.toRotationMatrix();
  rot_world_to_base = compute_world_to_base_rot(data, rot_world_to_imu, b_use_marg_filter);

  // compute estimator (control) input, u_n
  base_pose_model_.packAccelerationInput(rot_world_to_imu.transpose(),
                                         data->imu_lin_accel_, accelerometer_input_);

  // get initial base estimate
  Eigen::Vector3d world_to_base = robot_->GetLinkIsometry(draco_link::torso_link).translation()
                                  + rot_world_to_imu * robot_->GetRobotComPosLocal();

  // initialize Kalman filter state xhat =[0_pos_b, 0_vel_b, 0_pos_LF, 0_pos_RF]
  if (b_first_visit_) {
    x_hat_.initialize(world_to_base,
                      robot_->GetLinkIsometry(draco_link::l_foot_contact),
                      robot_->GetLinkIsometry(draco_link::r_foot_contact));
    kalman_filter_.init(x_hat_);
    b_first_visit_ = false;
  }

  // update contact
  if (data->b_rf_contact_) {
    sp_->b_rf_contact_ = true;
  } else {
    sp_->b_rf_contact_ = false;
  }

  if (data->b_lf_contact_) {
    sp_->b_lf_contact_ = true;
  } else {
    sp_->b_lf_contact_ = false;
  }
  updateSupportState(sp_, current_support_state_);

  // at support state change, update global offset and covariance gains
  if (current_support_state_ != prev_support_state_) {

    // from double to right support and viceversa
    if ((prev_support_state_ == DOUBLE) && (current_support_state_ == RIGHT)) {
      foot_pos_from_base_pre_transition = x_hat_.tail(6).head(3);
      base_pose_model_.update_leg_covariance(PoseMeasurementModel::LEFT, COV_LEVEL_HIGH);
    } else if ((prev_support_state_ == RIGHT) && (current_support_state_ == DOUBLE)) {
      foot_pos_from_base_post_transition = robot_->GetLinkIsometry(draco_link::l_foot_contact).translation();
      global_linear_offset_ = foot_pos_from_base_post_transition - foot_pos_from_base_pre_transition;
      global_linear_offset_.z() = 0.0;    // TODO make more robust later for non-flat ground
      system_model_.update_rfoot_offset(global_linear_offset_);
      base_pose_model_.update_leg_covariance(PoseMeasurementModel::LEFT, COV_LEVEL_LOW);
    } else if ((prev_support_state_ == DOUBLE) && (current_support_state_ == LEFT)) {
      // from double support to left support and viceversa
      foot_pos_from_base_pre_transition = x_hat_.tail(3);
      base_pose_model_.update_leg_covariance(PoseMeasurementModel::RIGHT, COV_LEVEL_HIGH);
    } else if ((prev_support_state_ == LEFT) && (current_support_state_ == DOUBLE)) {
      foot_pos_from_base_post_transition = robot_->GetLinkIsometry(draco_link::r_foot_contact).translation();
      global_linear_offset_ = foot_pos_from_base_post_transition - foot_pos_from_base_pre_transition;
      global_linear_offset_.z() = 0.0;  // TODO make more robust later for non-flat ground
      system_model_.update_lfoot_offset(global_linear_offset_);
      base_pose_model_.update_leg_covariance(PoseMeasurementModel::RIGHT, COV_LEVEL_LOW);
    }
    if (!foot_pos_from_base_post_transition.hasNaN()) {
      x_hat_ = kalman_filter_.predict(system_model_, accelerometer_input_);
      global_linear_offset_.setZero();
      system_model_.update_lfoot_offset(global_linear_offset_);
      system_model_.update_rfoot_offset(global_linear_offset_);
      b_skip_prediction = true;
    }
  }
  if(!b_skip_prediction)
    x_hat_ = kalman_filter_.predict(system_model_, accelerometer_input_);
  else
    b_skip_prediction = false;

//    this->_update_dcm();
  // update measurement assuming at least one foot is on the ground
  if (data->b_lf_contact_) {
    Eigen::Vector3d pos_base_from_lfoot = world_to_base - robot_->GetLinkIsometry(draco_link::l_foot_contact).translation();
    base_pose_model_.update_position_from_lfoot(pos_base_from_lfoot, base_estimate_);
  }
  if (data->b_rf_contact_) {
    Eigen::Vector3d pos_base_from_rfoot = world_to_base - robot_->GetLinkIsometry(draco_link::r_foot_contact).translation();
    base_pose_model_.update_position_from_rfoot(pos_base_from_rfoot, base_estimate_);
  }
  x_hat_ = kalman_filter_.update(base_pose_model_, base_estimate_);

  // values computed by linear KF estimator
  Eigen::Vector3d base_position_estimate( x_hat_.base_pos_x(),  x_hat_.base_pos_y(),  x_hat_.base_pos_z());
  Eigen::Vector3d base_velocity_estimate( x_hat_.base_vel_x(),  x_hat_.base_vel_y(),  x_hat_.base_vel_z());


//  robot_->update_system(
//          base_com_pos, margFilter_.getQuaternion(),
//          base_velocity_estimate, data->imu_frame_vel.head(3), base_position_estimate,
//          margFilter_.getQuaternion(), base_velocity_estimate,
//          data->imu_frame_vel.head(3), data->joint_positions,
//          data->joint_velocities, true);


  prev_support_state_ = current_support_state_;
  // save current time step data
  if (sp_->count_ % sp_->save_freq_ == 0) {
    DracoDataManager *dm = DracoDataManager::GetDataManager();
    dm->data_->base_pos_kf_ = base_position_estimate;
    dm->data_->base_vel_kf_ = base_velocity_estimate;
    dm->data_->base_euler_kf_ = util::QuatToEulerZYX(Eigen::Quaterniond(rot_world_to_base));
    dm->data_->base_quat_kf_ = Eigen::Vector4d(margFilter_.getQuaternion().w(),
                                             margFilter_.getQuaternion().x(),
                                             margFilter_.getQuaternion().y(),
                                             margFilter_.getQuaternion().z()) ;

    // save feet contact information
    dm->data_->lfoot_contact_ = sp_->stance_foot_ == draco_link::l_foot_contact;
    dm->data_->rfoot_contact_ = sp_->stance_foot_ == draco_link::r_foot_contact;
    dm->data_->lf_contact_ = sp_->b_lf_contact_;
    dm->data_->rf_contact_ = sp_->b_rf_contact_;

    dm->data_->base_com_pos_ = data->base_joint_pos_;    //TODO remove and pass directly from python
    dm->data_->base_com_quat_ = data->base_joint_quat_;  //TODO remove and pass directly from python
  }
}

void DracoKFStateEstimator::updateSupportState(DracoStateProvider* sp, SupportState& support_state)
{
  if (sp->b_rf_contact_ && sp->b_lf_contact_){
    support_state = DOUBLE;
    return;
  }

  if (sp->b_lf_contact_) {
    support_state = LEFT;
    return;
  }

  support_state = RIGHT;
}

Eigen::Matrix3d DracoKFStateEstimator::compute_world_to_base_rot(DracoSensorData *data,
                                                                 Eigen::Matrix3d rot_world_to_imu,
                                                                 bool use_marg_filter)
{
  if(use_marg_filter)
  {
      // Is imu_lin_vel or imu_ang_vel?
    margFilter_.filterUpdate(data->imu_ang_vel_[0], data->imu_ang_vel_[1], data->imu_ang_vel_[2],
                              data->imu_lin_accel_[0], data->imu_lin_accel_[1], data->imu_lin_accel_[2]);
    return margFilter_.getBaseRotation();
  }
  else
  {
    return rot_world_to_imu * iso_imu_to_base_com_.linear();
  }

}


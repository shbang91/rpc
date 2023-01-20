#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/filter/digital_filters.hpp"

#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_interface.hpp"
#include "controller/draco_controller/draco_state_estimator.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"

#include "util/util.hpp"

#if B_USE_ZMQ
#include "controller/draco_controller/draco_data_manager.hpp"
#endif

#include <string>

DracoStateEstimator::DracoStateEstimator(PinocchioRobotSystem *robot)
    : robot_(robot), R_imu_base_com_(Eigen::Matrix3d::Identity()),
      global_leg_odometry_(Eigen::Vector3d::Zero()),
      prev_base_joint_pos_(Eigen::Vector3d::Zero()), b_first_visit_(true) {
  util::PrettyConstructor(1, "DracoStateEstimator");
  sp_ = DracoStateProvider::GetStateProvider();

  R_imu_base_com_ =
      robot_->GetLinkIsometry(draco_link::torso_imu).linear().transpose() *
      robot_->GetLinkIsometry(draco_link::torso_com_link).linear();

  try {
    YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
    com_vel_filter_type_ =
        util::ReadParameter<int>(cfg["state_estimator"], "com_vel_filter_type");
    bool b_sim = util::ReadParameter<bool>(cfg, "b_sim");

    std::string prefix = b_sim ? "sim" : "exp";
    if (com_vel_filter_type_ == com_vel_filter::kMovingAverage) {
      Eigen::Vector3d num_data_com_vel = util::ReadParameter<Eigen::Vector3d>(
          cfg["state_estimator"], prefix + "_num_data_com_vel");
      com_vel_mv_avg_filter_.clear();
      for (int i = 0; i < 3; i++) {
        com_vel_mv_avg_filter_.push_back(
            new SimpleMovingAverage(num_data_com_vel[i]));
      }
    } else if (com_vel_filter_type_ == com_vel_filter::kExponentialSmoother) {
      double time_constant = util::ReadParameter<double>(
          cfg["state_estimator"], "com_vel_time_constant");
      Eigen::VectorXd com_vel_err_limit = util::ReadParameter<Eigen::VectorXd>(
          cfg["state_estimator"], "com_vel_err_limit");
      com_vel_exp_filter_ = new ExponentialMovingAverageFilter(
          sp_->servo_dt_, time_constant, Eigen::VectorXd::Zero(3),
          -com_vel_err_limit, com_vel_err_limit);
    } else if (com_vel_filter_type_ == com_vel_filter::kLowPassFilter) {
      double cut_off_period =
          util::ReadParameter<double>(cfg["state_estimator"], "cut_off_period");
      com_vel_lp_filter_ =
          new LowPassVelocityFilter(sp_->servo_dt_, cut_off_period, 3);
    }

  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }

#if B_USE_MATLOGGER
  logger_ = XBot::MatLogger2::MakeLogger("/tmp/draco_state_estimator_data");
#endif
}

DracoStateEstimator::~DracoStateEstimator() {
  while (!com_vel_mv_avg_filter_.empty()) {
    delete com_vel_mv_avg_filter_.back();
    com_vel_mv_avg_filter_.pop_back();
  }
  if (com_vel_exp_filter_ != nullptr)
    delete com_vel_exp_filter_;
}

void DracoStateEstimator::Initialize(DracoSensorData *sensor_data) {
  robot_->UpdateRobotModel(
      Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(),
      Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), sensor_data->joint_pos_,
      sensor_data->joint_vel_, false);
}

void DracoStateEstimator::Update(DracoSensorData *sensor_data) {

  // Estimate floating base orientation
  Eigen::Quaterniond imu_quat(
      sensor_data->imu_frame_quat_[3], sensor_data->imu_frame_quat_[0],
      sensor_data->imu_frame_quat_[1], sensor_data->imu_frame_quat_[2]);
  Eigen::Matrix3d base_joint_ori_rot =
      imu_quat.normalized().toRotationMatrix() * R_imu_base_com_;

  // TODO imu angular velocity filtering for real experiment -> PnC does not use
  // this in controller

  // Update robot model only with base orientation
  robot_->UpdateRobotModel(Eigen::Vector3d::Zero(),
                           Eigen::Quaterniond(base_joint_ori_rot).normalized(),
                           Eigen::Vector3d::Zero(), sensor_data->imu_ang_vel_,
                           sensor_data->joint_pos_, sensor_data->joint_vel_,
                           false);

  // Estimate floating base position
  // anchor frame depending on the stance foot
  Eigen::Vector3d anchor_frame_pos =
      robot_->GetLinkIsometry(sp_->stance_foot_).translation();
  Eigen::Vector3d anchor_frame_vel =
      robot_->GetLinkSpatialVel(sp_->stance_foot_).tail<3>();
  if (sp_->stance_foot_ != sp_->prev_stance_foot_) {
    Eigen::Vector3d anchor_frame_pos_diff =
        anchor_frame_pos -
        robot_->GetLinkIsometry(sp_->prev_stance_foot_).translation();
    global_leg_odometry_ += anchor_frame_pos_diff;
  }

  // estimate position
  Eigen::Vector3d base_joint_pos = global_leg_odometry_ - anchor_frame_pos;
  if (b_first_visit_) {
    prev_base_joint_pos_ = base_joint_pos;
    b_first_visit_ = false;
  }

  // estimate base linear velocity
  Eigen::Vector3d base_joint_lin_vel =
      (base_joint_pos - prev_base_joint_pos_) / sp_->servo_dt_;

  // save current time step data
  sp_->prev_stance_foot_ = sp_->stance_foot_;
  prev_base_joint_pos_ = base_joint_pos;

  // Update robot model with full estimate
  robot_->UpdateRobotModel(
      base_joint_pos, Eigen::Quaterniond(base_joint_ori_rot).normalized(),
      base_joint_lin_vel, sensor_data->imu_ang_vel_, sensor_data->joint_pos_,
      sensor_data->joint_vel_, true);

  // TODO:foot contact switch
  // sp_->b_lf_contact_ = (sensor_data->b_lf_contact_) ? true : false;
  // sp_->b_rf_contact_ = (sensor_data->b_rf_contact_) ? true : false;

  // velocity filtering for com vel (for real experiment)
  Eigen::Vector3d real_com_vel = robot_->GetRobotComLinVel();
  if (com_vel_filter_type_ == com_vel_filter::kMovingAverage) {
    for (int i = 0; i < 3; ++i) {
      com_vel_mv_avg_filter_[i]->Input(real_com_vel[i]);
      sp_->com_vel_est_[i] = com_vel_mv_avg_filter_[i]->Output();
    }
  } else if (com_vel_filter_type_ == com_vel_filter::kExponentialSmoother) {
    Eigen::VectorXd real_com_vel_xd(3);
    real_com_vel_xd << real_com_vel;
    com_vel_exp_filter_->Input(real_com_vel_xd);
    Eigen::VectorXd output = com_vel_exp_filter_->Output();
    sp_->com_vel_est_ << output[0], output[1], output[2];
  } else if (com_vel_filter_type_ == com_vel_filter::kLowPassFilter) {
    Eigen::VectorXd com_pos(3);
    com_pos << robot_->GetRobotComPos();

    if (b_lp_first_visit_) {
      com_vel_lp_filter_->Reset(com_pos);
      b_lp_first_visit_ = false;
    }

    com_vel_lp_filter_->Input(com_pos);
    Eigen::VectorXd output = com_vel_lp_filter_->Output();
    sp_->com_vel_est_ << output[0], output[1], output[2];
  }

#if B_USE_ZMQ
  // Save estimated floating base joint states
  DracoDataManager *dm = DracoDataManager::GetDataManager();
  dm->data_->est_base_joint_pos_ = base_joint_pos;
  Eigen::Quaterniond base_joint_quat(base_joint_ori_rot);
  dm->data_->est_base_joint_ori_ << base_joint_quat.normalized().coeffs();

  // Save joint pos data
  dm->data_->joint_positions_ = sensor_data->joint_pos_;

  dm->data_->est_icp = sp_->dcm_.head<2>();
  // for simulation only (ground truth data from simulator)
  // dm->data_->base_joint_pos_ = sensor_data->base_joint_pos_;
  // dm->data_->base_joint_ori_ = sensor_data->base_joint_quat_;
  // dm->data_->base_joint_lin_vel_ = sensor_data->base_joint_lin_vel_;
  // dm->data_->base_joint_ang_vel_ = sensor_data->base_joint_ang_vel_;
#endif

  // compute dcm
  this->_ComputeDCM();

  // save data
#if B_USE_MATLOGGER
  if (sp_->count_ % sp_->data_save_freq_ == 0) {
    // joint encoder data
    logger_->add("joint_pos_act", sensor_data->joint_pos_);
    logger_->add("joint_vel_act", sensor_data->joint_vel_);

    // floating base estimate data
    logger_->add("base_joint_pos_est", base_joint_pos);
    logger_->add("base_joint_rpy_est", util::RPYFromSO3(base_joint_ori_rot));
    logger_->add("base_joint_lin_vel_est", base_joint_lin_vel);
    logger_->add("base_joint_ang_vel_est", sensor_data->imu_ang_vel_);

    // com velocities
    logger_->add("com_vel_raw", real_com_vel);
    logger_->add("com_vel_est", sp_->com_vel_est_);

    // icp data
    logger_->add("icp_est", sp_->dcm_.head<2>());
    logger_->add("icp_vel_est", sp_->dcm_vel_.head<2>());

    // TODO: imu ang vel
  }
#endif
}

void DracoStateEstimator::_ComputeDCM() {
  Eigen::Vector3d com_pos = robot_->GetRobotComPos();
  Eigen::Vector3d com_vel = sp_->com_vel_est_; // TODO: use filtered com vel
  double omega = sqrt(9.81 / com_pos[2]);

  sp_->prev_dcm_ = sp_->dcm_;
  sp_->dcm_ = com_pos + com_vel / omega;

  double cutoff_period =
      0.01; // 10ms cut-off period for first-order low pass filter
  double alpha = sp_->servo_dt_ / cutoff_period;

  sp_->dcm_vel_ = alpha * (sp_->dcm_ - sp_->prev_dcm_) / sp_->servo_dt_ +
                  (1 - alpha) * sp_->dcm_vel_; // not being used in controller
}

void DracoStateEstimator::UpdateGroundTruthSensorData(
    DracoSensorData *sensor_data) {
  Eigen::Vector4d base_joint_ori = sensor_data->base_joint_quat_;
  Eigen::Quaterniond base_joint_quat(base_joint_ori[3], base_joint_ori[0],
                                     base_joint_ori[1], base_joint_ori[2]);

  robot_->UpdateRobotModel(
      sensor_data->base_joint_pos_, base_joint_quat.normalized(),
      sensor_data->base_joint_lin_vel_, sensor_data->base_joint_ang_vel_,
      sensor_data->joint_pos_, sensor_data->joint_vel_, true);

  this->_ComputeDCM();

#if B_USE_ZMQ
  // DracoDataManager *dm = DracoDataManager::GetDataManager();
  // dm->data_->base_joint_pos_ = sensor_data->base_joint_pos_;
  // dm->data_->base_joint_ori_ = sensor_data->base_joint_quat_;
  // dm->data_->base_joint_lin_vel_ = sensor_data->base_joint_lin_vel_;
  // dm->data_->base_joint_ang_vel_ = sensor_data->base_joint_ang_vel_;

  // dm->data_->joint_positions_ = sensor_data->joint_pos_;
#endif
}

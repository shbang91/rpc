#include "contact_detection_manager.hpp"

#include "util/util.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"

ContactDetectionManager::ContactDetectionManager(PinocchioRobotSystem *_robot)
  : b_contact_touchdown_(2, false), b_heel_toe_touchdown_(2, false) {
  //util::PrettyConstructor(2, "ContactDetectionManager");

  robot_ = _robot;
  wait_count_post_transition_ = 100;    // wait this many control ticks before re-transitioning
  count_post_transition_ = 0;

  try {
    YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");

    bool b_sim = util::ReadParameter<bool>(cfg, "b_sim");
    std::string prefix = b_sim ? "sim" : "exp";

    b_debug_only_log_ = util::ReadParameter<bool>(
            cfg["contact_detection"], "b_debug_only_log");
    b_use_contact_sensor_ = util::ReadParameter<bool>(
            cfg["contact_detection"], "b_use_contact_sensor");
    b_use_foot_height_ = util::ReadParameter<bool>(
            cfg["contact_detection"], "b_use_foot_height");
    auto servo_dt = util::ReadParameter<double>(cfg, "servo_dt");
    auto contact_time_constant = util::ReadParameter<double>(
            cfg["contact_detection"], prefix + "_contact_time_constant");
    auto contact_limits = util::ReadParameter<double>(
            cfg["wbc"]["contact"], prefix + "_max_rf_z");
    auto foot_half_width = util::ReadParameter<double>(
            cfg["wbc"]["contact"], prefix + "_foot_half_width");
    auto foot_half_length = util::ReadParameter<double>(
            cfg["wbc"]["contact"], prefix + "_foot_half_length");
    foot_height_tol_ = util::ReadParameter<double>(
            cfg["contact_detection"], prefix + "_foot_height_tol");
    schmitt_thresholds_ = util::ReadParameter<Eigen::Vector2d>(
            cfg["contact_detection"], prefix + "_schmitt_thresholds");
    volt_to_force_map_ = util::ReadParameter<Eigen::Vector2d>(
            cfg["contact_detection"], "volt_to_force_map");
    volt_bias_ = util::ReadParameter<Eigen::Vector2d>(
            cfg["contact_detection"], "volt_bias");

    Eigen::Vector2d contact_limits_vec;
    contact_limits_vec << contact_limits, contact_limits_vec;
    contact_sensor_filter_ = std::make_unique<ExponentialMovingAverageFilter>(
            servo_dt, contact_time_constant, Eigen::VectorXd::Zero(2),
            -contact_limits_vec, contact_limits_vec);

    // get first swing leg side
    auto b_swing_side = util::ReadParameter<int>(
            cfg["dcm_walking"], "first_swing_leg");
    if (b_swing_side == end_effector::RFoot) {
      swing_leg_name_ = draco_link::r_foot_contact;
      support_leg_name_ = draco_link::l_foot_contact;
    } else {
      swing_leg_name_ = draco_link::l_foot_contact;
      support_leg_name_ = draco_link::r_foot_contact;
    }

    // set up feet corner point local positions w.r.t. ankle
    Eigen::Vector2d l_front_pos = Eigen::Vector2d(foot_half_length, foot_half_width);
    Eigen::Vector2d r_front_pos = Eigen::Vector2d(foot_half_length, -foot_half_width);
    Eigen::Vector2d r_back_pos = Eigen::Vector2d(-foot_half_length, -foot_half_width);
    Eigen::Vector2d l_back_pos = Eigen::Vector2d(-foot_half_length, foot_half_width);
    foot_corner_map_.insert(std::pair<int, Eigen::Vector2d>(0, l_front_pos));
    foot_corner_map_.insert(std::pair<int, Eigen::Vector2d>(1, r_front_pos));
    foot_corner_map_.insert(std::pair<int, Eigen::Vector2d>(2, r_back_pos));
    foot_corner_map_.insert(std::pair<int, Eigen::Vector2d>(3, l_back_pos));

  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }

  contact_forces_raw_.setZero();
  contact_forces_filt_.setZero();
}

ContactDetectionManager::~ContactDetectionManager() {}

void ContactDetectionManager::_UpdateSwingSide(DracoStateProvider *_sp) {

  // if swing leg changed, update leg names accordingly
  if (_sp->b_swing_leg_ == end_effector::LFoot) {
    swing_leg_name_ = draco_link::l_foot_contact;
    support_leg_name_ = draco_link::r_foot_contact;
  } else {
    swing_leg_name_ = draco_link::r_foot_contact;
    support_leg_name_ = draco_link::l_foot_contact;
  }

  _sp->b_request_change_swing_leg_ = false;
  count_post_transition_ = 0;
  std::cout << "Applied swing leg change to " << _sp->b_swing_leg_ << std::endl;
}

void ContactDetectionManager::_CheckSwingFootContact(const double expected_height_difference) {

  // get swing and support position and orientation
  Eigen::Vector3d swing_foot_pos = robot_->GetLinkIsometry(swing_leg_name_).translation();
  Eigen::Matrix3d swing_foot_ori = robot_->GetLinkIsometry(swing_leg_name_).linear();
  Eigen::Vector3d support_foot_pos = robot_->GetLinkIsometry(support_leg_name_).translation();
  Eigen::Matrix3d support_foot_ori = robot_->GetLinkIsometry(support_leg_name_).linear();

  double ankle_to_corner_height = 0.;
  double swing_foot_corner_height = 0.;
  bool foot_has_touchdown = false;
  for(auto const &[num, pos] : foot_corner_map_) {
    ankle_to_corner_height = swing_foot_ori(2,0) * pos(0) + swing_foot_ori(2,1) * pos(1);
    swing_foot_corner_height = swing_foot_pos.z() + ankle_to_corner_height;
    foot_has_touchdown = (swing_foot_corner_height - support_foot_pos.z())
            < expected_height_difference + foot_height_tol_;

    // if either heel or toe have touched down, terminate
    if(foot_has_touchdown) {
      // update the swing leg
      if (swing_leg_name_ == draco_link::l_foot_contact) {
        b_heel_toe_touchdown_[end_effector::LFoot] = true;
      } else if (swing_leg_name_ == draco_link::r_foot_contact) {
        b_heel_toe_touchdown_[end_effector::RFoot] = true;
      }
      return;
    }
  }

  // if we reach this point, the foot has kinematically NOT touched the ground
  if (swing_leg_name_ == draco_link::l_foot_contact) {
    b_heel_toe_touchdown_[end_effector::LFoot] = false;
  } else if (swing_leg_name_ == draco_link::r_foot_contact) {
    b_heel_toe_touchdown_[end_effector::RFoot] = false;
  }

}

void ContactDetectionManager::_ConvertVoltageToForce(const Eigen::Vector2d &contact_sensor_v,
                                                     Eigen::Vector2d &contact_force_return) {
  contact_force_return = volt_to_force_map_.cwiseProduct(contact_sensor_v) + volt_bias_;
}

void ContactDetectionManager::_CheckContactThresholds(const DracoStateProvider *_sp) {
  // Schmitt trigger

  // Left foot (add !_sp->b_lf_contact_ for ONLY early contact)
  if ((contact_forces_filt_(end_effector::LFoot) < schmitt_thresholds_(LimitsIdx::low))) {
    // if previously in contact and LOWER force threshold reached, change to not in contact
    b_contact_touchdown_[end_effector::LFoot] = false;
  } else if ((contact_forces_filt_(end_effector::LFoot) > schmitt_thresholds_(LimitsIdx::high))) {
    // if previously NOT in contact and UPPER force threshold reached, change to in contact
    b_contact_touchdown_[end_effector::LFoot] = true;
  }

  // Right foot
  if ((contact_forces_filt_(end_effector::RFoot) < schmitt_thresholds_(LimitsIdx::low))) {
    // if previously in contact and LOWER force threshold reached, change to not in contact
    b_contact_touchdown_[end_effector::RFoot] = false;
  } else if ((contact_forces_filt_(end_effector::RFoot) > schmitt_thresholds_(LimitsIdx::high))) {
    // if previously NOT in contact and UPPER force threshold reached, change to not in contact
    b_contact_touchdown_[end_effector::RFoot] = true;
  }
}

void ContactDetectionManager::UpdateForceMeasurements(const Eigen::Vector2d &contact_normal_raw) {
  _ConvertVoltageToForce(contact_normal_raw, contact_forces_raw_);

  // filter forces
  contact_sensor_filter_->Input(contact_forces_raw_);
  contact_forces_filt_ = contact_sensor_filter_->Output();
}

bool ContactDetectionManager::UpdateContactStates(DracoStateProvider *_sp,
                            const Eigen::Vector2d &_expected_contact_height) {

  if (b_use_contact_sensor_)
    _CheckContactThresholds(_sp);

  if (b_use_foot_height_) {
    if (_sp->b_request_change_swing_leg_)
      _UpdateSwingSide(_sp);

    // check kinematics when left foot is swing foot
    if (swing_leg_name_ == draco_link::l_foot_contact) {
      _CheckSwingFootContact(_expected_contact_height(end_effector::LFoot));
    }

    // check kinematics when right foot is swing foot
    if (swing_leg_name_ == draco_link::r_foot_contact) {
      _CheckSwingFootContact(_expected_contact_height(end_effector::RFoot));
    }
  }

  // Update contact states on StateProvider in case of early touch-down event
  if (!b_debug_only_log_ && count_post_transition_ > wait_count_post_transition_) {
    // Check only swing leg
    if (swing_leg_name_ == draco_link::l_foot_contact) {
      if (!_sp->b_lf_contact_ && (b_contact_touchdown_[end_effector::LFoot]
                                  || b_heel_toe_touchdown_[end_effector::LFoot])) {
        _sp->b_lf_contact_ = true;
        std::cout << "[Contact Detection Manager] Early LF contact" << std::endl;
      }
    } else if (swing_leg_name_ == draco_link::r_foot_contact) {
      if (!_sp->b_rf_contact_ && (b_contact_touchdown_[end_effector::RFoot]
                                  || b_heel_toe_touchdown_[end_effector::RFoot])) {
        _sp->b_rf_contact_ = true;
        std::cout << "[Contact Detection Manager] Early RF contact" << std::endl;
      }
    }
  }

  count_post_transition_++;
}

double ContactDetectionManager::GetLFootNormalForceRaw() {
  return contact_forces_raw_(end_effector::LFoot);
}

double ContactDetectionManager::GetRFootNormalForceRaw() {
  return contact_forces_raw_(end_effector::RFoot);
}

double ContactDetectionManager::GetFootNormalForceFilt(int _end_effector) {
  return contact_forces_filt_(_end_effector);
}

bool ContactDetectionManager::HasContactSensorTouchdown(int _end_effector) {
  return b_contact_touchdown_[_end_effector];
}

bool ContactDetectionManager::HasHeelToeTouchdown(int _end_effector) {
  return b_heel_toe_touchdown_[_end_effector];
}


#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "util/util.hpp"

PinocchioRobotSystem::PinocchioRobotSystem(const std::string &urdf_file,
                                           const std::string &package_dir,
                                           const bool b_fixed_base,
                                           const bool b_print_info)
    : urdf_file_(urdf_file), package_dir_(package_dir),
      b_fixed_base_(b_fixed_base), b_print_info_(b_print_info) {
  util::PrettyConstructor(1, "PinocchioRobotSystem");
  this->_Initialize();
  if (b_print_info)
    this->_PrintRobotInfo();
}

void PinocchioRobotSystem::_Initialize() {
  if (b_fixed_base_) {
    pinocchio::urdf::buildModel(urdf_file_, model_);
    pinocchio::urdf::buildGeom(model_, urdf_file_, pinocchio::COLLISION,
                               collision_model_, package_dir_);
    pinocchio::urdf::buildGeom(model_, urdf_file_, pinocchio::VISUAL,
                               visual_model_, package_dir_);
    n_float_ = 0;
  } else {
    pinocchio::urdf::buildModel(urdf_file_, pinocchio::JointModelFreeFlyer(),
                                model_);
    pinocchio::urdf::buildGeom(model_, urdf_file_, pinocchio::COLLISION,
                               collision_model_, package_dir_);
    pinocchio::urdf::buildGeom(model_, urdf_file_, pinocchio::VISUAL,
                               visual_model_, package_dir_);
    n_float_ = 6;
  }

  data_ = pinocchio::Data(model_);
  collision_data_ = pinocchio::GeometryData(collision_model_);
  visual_data_ = pinocchio::GeometryData(visual_model_);

  n_q_ = model_.nq;
  n_qdot_ = model_.nv;
  n_adof_ = n_qdot_ - n_float_; // note that virtual dof is not considered here

  for (pinocchio::FrameIndex i(0);
       i < static_cast<pinocchio::FrameIndex>(model_.nframes); ++i) {
    std::string frame_name = model_.frames[i].name;
    if (frame_name != "universe" && frame_name != "root_joint")
      if (i % 2 == 0)
        link_idx_map_[model_.getFrameId(frame_name)] = frame_name;
  }
  // std::cout << "link_idx_map" << std::endl;
  // for (const auto &pair : link_idx_map_)
  // std::cout << pair.first << ":" << pair.second << std::endl;

  for (pinocchio::JointIndex i(0);
       i < static_cast<pinocchio::JointIndex>(model_.njoints); ++i) {
    total_mass_ += model_.inertias[i].mass();
    std::string joint_name = model_.names[i];
    if (joint_name != "universe" && joint_name != "root_joint")
      joint_idx_map_[i - 2] = joint_name; // joint map excluding fixed joint
  }
  assert(n_adof_ == joint_idx_map_.size());

  joint_pos_limits_.resize(n_adof_, 2);
  joint_vel_limits_.resize(n_adof_, 2);
  joint_trq_limits_.resize(n_adof_, 2);

  if (b_fixed_base_) {
    joint_pos_limits_.leftCols(1) = model_.lowerPositionLimit;
    joint_pos_limits_.rightCols(1) = model_.upperPositionLimit;
    joint_vel_limits_.leftCols(1) = -model_.velocityLimit;
    joint_vel_limits_.rightCols(1) = model_.velocityLimit;
    joint_trq_limits_.leftCols(1) = -model_.effortLimit;
    joint_trq_limits_.rightCols(1) = model_.effortLimit;
  } else {
    joint_pos_limits_.leftCols(1) =
        model_.lowerPositionLimit.segment(n_float_, n_adof_);
    joint_pos_limits_.rightCols(1) =
        model_.upperPositionLimit.segment(n_float_, n_adof_);
    joint_vel_limits_.leftCols(1) =
        -model_.velocityLimit.segment(n_float_, n_adof_);
    joint_vel_limits_.rightCols(1) =
        model_.velocityLimit.segment(n_float_, n_adof_);
    joint_trq_limits_.leftCols(1) =
        -model_.effortLimit.segment(n_float_, n_adof_);
    joint_trq_limits_.rightCols(1) =
        model_.effortLimit.segment(n_float_, n_adof_);
  }

  q_.resize(n_q_);
  qdot_.resize(n_qdot_);
  Ig_.setZero();
  Hg_.setZero();
  Ag_.resize(6, n_qdot_);
}

void PinocchioRobotSystem::UpdateRobotModel(
    const Eigen::Vector3d &base_joint_pos,
    const Eigen::Quaterniond &base_joint_quat,
    const Eigen::Vector3d &base_joint_lin_vel,
    const Eigen::Vector3d &base_joint_ang_vel, const Eigen::VectorXd &joint_pos,
    const Eigen::VectorXd &joint_vel, bool b_update_centroid) {
  if (!b_fixed_base_) {
    q_.segment(0, 3) = base_joint_pos;
    q_.segment(3, 4) << base_joint_quat.normalized().coeffs();
    q_.tail(n_q_ - 7) = joint_pos;

    Eigen::Matrix3d rot_w_basejoint =
        base_joint_quat.normalized().toRotationMatrix();
    qdot_.segment(0, 3) = rot_w_basejoint.transpose() * base_joint_lin_vel;
    qdot_.segment(3, 3) = rot_w_basejoint.transpose() * base_joint_ang_vel;
    qdot_.tail(n_qdot_ - n_float_) = joint_vel;

  } else {
    // fixed base robot
    q_ = joint_pos;
    qdot_ = joint_vel;
  }
  pinocchio::forwardKinematics(model_, data_, q_, qdot_);

  if (b_update_centroid)
    this->_UpdateCentroidalQuantities();
}

void PinocchioRobotSystem::_UpdateCentroidalQuantities() {
  pinocchio::ccrba(model_, data_, q_, qdot_);

  Ig_.block<3, 3>(0, 0) = data_.Ig.matrix().block<3, 3>(3, 3);
  Ig_.block<3, 3>(3, 3) = data_.Ig.matrix().block<3, 3>(0, 0);

  Hg_.segment(0, 3) = data_.hg.angular();
  Hg_.segment(3, 3) = data_.hg.linear();

  Ag_.topRows(3) = data_.Ag.bottomRows(3);
  Ag_.bottomRows(3) = data_.Ag.topRows(3);
}

// Kinematics getter
Eigen::VectorXd PinocchioRobotSystem::GetQ() const { return this->q_; }
Eigen::VectorXd PinocchioRobotSystem::GetQdot() const { return this->qdot_; }

int PinocchioRobotSystem::GetQIdx(const int joint_idx) const {
  int idx = 7 + joint_idx;
  return idx;
}

int PinocchioRobotSystem::GetQdotIdx(const int joint_idx) const {
  int idx = 6 + joint_idx;
  return idx;
}

Eigen::VectorXd PinocchioRobotSystem::GetJointPos() const {
  return this->q_.tail(n_adof_);
}
Eigen::VectorXd PinocchioRobotSystem::GetJointVel() const {
  return this->qdot_.tail(n_adof_);
}

Eigen::Isometry3d PinocchioRobotSystem::GetLinkIsometry(const int link_idx) {
  Eigen::Isometry3d ret;
  const pinocchio::SE3 trans =
      pinocchio::updateFramePlacement(model_, data_, link_idx);
  ret.linear() = trans.rotation();
  ret.translation() = trans.translation();
  return ret;
}

Eigen::Matrix<double, 6, 1>
PinocchioRobotSystem::GetLinkSpatialVel(const int link_idx) const {
  Eigen::Matrix<double, 6, 1> ret = Eigen::Matrix<double, 6, 1>::Zero();
  pinocchio::Motion fv = pinocchio::getFrameVelocity(
      model_, data_, link_idx, pinocchio::LOCAL_WORLD_ALIGNED);
  ret.head<3>() = fv.angular();
  ret.tail<3>() = fv.linear();
  return ret;
}

Eigen::Matrix<double, 6, Eigen::Dynamic>
PinocchioRobotSystem::GetLinkJacobian(const int link_idx) {
  pinocchio::computeJointJacobians(model_, data_, q_);

  Eigen::Matrix<double, 6, Eigen::Dynamic> jac =
      Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, n_qdot_);
  pinocchio::getFrameJacobian(model_, data_, link_idx,
                              pinocchio::LOCAL_WORLD_ALIGNED, jac);
  Eigen::Matrix<double, 6, Eigen::Dynamic> ret =
      Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, n_qdot_);
  ret.topRows(3) = jac.bottomRows(3);
  ret.bottomRows(3) = jac.topRows(3);
  return ret;
}

Eigen::Matrix<double, 6, 1>
PinocchioRobotSystem::GetLinkJacobianDotQdot(const int link_idx) {
  pinocchio::forwardKinematics(model_, data_, q_, qdot_, 0 * qdot_);
  pinocchio::Motion fa = pinocchio::getFrameClassicalAcceleration(
      model_, data_, link_idx, pinocchio::LOCAL_WORLD_ALIGNED);

  Eigen::Matrix<double, 6, 1> ret = Eigen::Matrix<double, 6, 1>::Zero();
  ret.segment(0, 3) = fa.angular();
  ret.segment(3, 3) = fa.linear();

  return ret;
}

Eigen::Matrix<double, 6, 1>
PinocchioRobotSystem::GetLinkBodySpatialVel(const int link_idx) const {
  Eigen::Matrix<double, 6, 1> ret = Eigen::Matrix<double, 6, 1>::Zero();
  pinocchio::Motion fv =
      pinocchio::getFrameVelocity(model_, data_, link_idx, pinocchio::LOCAL);
  ret.segment(0, 3) = fv.angular();
  ret.segment(3, 3) = fv.linear();
  return ret;
}

Eigen::Matrix<double, 6, Eigen::Dynamic>
PinocchioRobotSystem::GetLinkBodyJacobian(const int link_idx) {
  pinocchio::computeJointJacobians(model_, data_, q_);

  Eigen::Matrix<double, 6, Eigen::Dynamic> jac =
      Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, n_qdot_);
  pinocchio::getFrameJacobian(model_, data_, link_idx, pinocchio::LOCAL, jac);
  Eigen::Matrix<double, 6, Eigen::Dynamic> ret =
      Eigen::Matrix<double, 6, Eigen::Dynamic>::Zero(6, n_qdot_);
  ret.topRows(3) = jac.bottomRows(3);
  ret.bottomRows(3) = jac.topRows(3);
  return ret;
}

Eigen::Matrix<double, 6, 1>
PinocchioRobotSystem::GetLinkBodyJacobianDotQdot(const int link_idx) {
  pinocchio::forwardKinematics(model_, data_, q_, qdot_, 0 * qdot_);
  pinocchio::Motion fa = pinocchio::getFrameClassicalAcceleration(
      model_, data_, link_idx, pinocchio::LOCAL);

  Eigen::Matrix<double, 6, 1> ret = Eigen::Matrix<double, 6, 1>::Zero();
  ret.segment(0, 3) = fa.angular();
  ret.segment(3, 3) = fa.linear();

  return ret;
}

Eigen::Vector3d PinocchioRobotSystem::GetRobotComPos() {
  return pinocchio::centerOfMass(model_, data_, q_);
}
Eigen::Vector3d PinocchioRobotSystem::GetRobotComLinVel() {
  pinocchio::centerOfMass(model_, data_, q_, qdot_);
  return data_.vcom[0];
}

Eigen::Matrix<double, 3, Eigen::Dynamic>
PinocchioRobotSystem::GetComLinJacobian() {
  return jacobianCenterOfMass(model_, data_, q_);
}

Eigen::Matrix<double, 3, 1> PinocchioRobotSystem::GetComLinJacobianDotQdot() {
  return (computeCentroidalMapTimeVariation(model_, data_, q_, qdot_)
              .topRows(3)) /
         total_mass_ * qdot_;
}

// dynamics getter
Eigen::MatrixXd PinocchioRobotSystem::GetMassMatrix() {
  pinocchio::crba(model_, data_, q_);
  data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  return data_.M;
}
Eigen::VectorXd PinocchioRobotSystem::GetGravity() {
  return pinocchio::computeGeneralizedGravity(model_, data_, q_);
}
Eigen::VectorXd PinocchioRobotSystem::GetCoriolis() {
  return pinocchio::nonLinearEffects(model_, data_, q_, qdot_) -
         pinocchio::computeGeneralizedGravity(model_, data_, q_);
}

Eigen::Matrix<double, 6, 6> PinocchioRobotSystem::GetIg() const {
  return this->Ig_;
}
Eigen::Matrix<double, 6, 1> PinocchioRobotSystem::GetHg() const {
  return this->Hg_;
}
Eigen::Matrix<double, 6, Eigen::Dynamic> PinocchioRobotSystem::GetAg() const {
  return this->Ag_;
}

int PinocchioRobotSystem::NumQdot() const { return this->n_qdot_; }
int PinocchioRobotSystem::NumActiveDof() const { return this->n_adof_; }
int PinocchioRobotSystem::NumFloatDof() const { return this->n_float_; }

// Print robot info
void PinocchioRobotSystem::_PrintRobotInfo() {
  std::cout << "=======================" << std::endl;
  std::cout << "Pinocchio robot info" << std::endl;
  std::cout << "=======================" << std::endl;

  std::cout << "============ draco link ================" << std::endl;
  // for (pinocchio::FrameIndex i = 1;
  // i < static_cast<pinocchio::FrameIndex>(model_.nframes); ++i) {
  // std::cout << "constexpr int " << model_.frames[i].name << " = "
  //<< model_.getFrameId(model_.frames[i].name) << ";" << std::endl;
  //}
  for (auto iter = link_idx_map_.begin(); iter != link_idx_map_.end(); ++iter) {
    std::cout << "constexpr int " << iter->second << " = " << iter->first << ";"
              << std::endl;
  }
  std::cout << "============ draco joint ================" << std::endl;
  // for (pinocchio::JointIndex i = 1;
  // i < static_cast<pinocchio::JointIndex>(model_.njoints); ++i) {
  // std::cout << "constexpr int " << model_.names[i] << " = "
  //<< model_.getJointId(model_.names[i]) << ";" << std::endl;
  //}
  for (auto iter = joint_idx_map_.begin(); iter != joint_idx_map_.end();
       ++iter) {
    std::cout << "constexpr int " << iter->second << " = " << iter->first << ";"
              << std::endl;
  }

  std::cout << "============ draco ================" << std::endl;
  std::cout << "constexpr int n_qdot = " << qdot_.size() << ";" << std::endl;
  std::cout << "constexpr int n_adof = " << qdot_.size() - n_float_ << ";"
            << std::endl;
  exit(0);
}

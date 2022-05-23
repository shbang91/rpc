#include "pnc/robot_system/pinocchio_robot_system.hpp"

PinocchioRobotSystem::PinocchioRobotSystem(const std::string &_urdf_file,
                                           const std::string &_package_dir,
                                           const bool _b_fixed_base,
                                           const bool _b_print_info,
                                           const int _num_virtual_dof)
    : RobotSystem(_b_fixed_base, _b_print_info), urdf_file_(_urdf_file),
      package_dir_(_package_dir), n_vdof_(_num_virtual_dof) {

  this->ConfigRobot();

  joint_positions_.resize(n_a_);
  joint_velocities_.resize(n_a_);
  q_ = Eigen::VectorXd::Zero(n_q_);
  qdot_ = Eigen::VectorXd::Zero(n_qdot_);

  Ag_.resize(6, n_qdot_);

  if (b_print_info_) {
    this->PrintRobotInfo();
  }
}

void PinocchioRobotSystem::ConfigRobot() {

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
  n_a_ = n_qdot_ - n_float_;

  int passing_index = 0;
  for (pinocchio::JointIndex i = 0;
       i < static_cast<pinocchio::JointIndex>(model_.njoints);
       ++i) { // NOT SURE IF START AT 0???
    if (model_.names[i] == "root_joint" || model_.names[i] == "universe") {
      passing_index += 1;
    } else {
      joint_id_[model_.names[i]] = i - passing_index;
    }
  }

  for (pinocchio::FrameIndex i = 0;
       i < static_cast<pinocchio::FrameIndex>(model_.nframes); ++i) {
    std::string frameName = model_.frames[i].name;
    if (frameName == "root_joint" || frameName == "universe") {
    } // pass
    else {
      if (i % 2 == 0) {
        // Link
        int link_id = floor(i / 2 - 1);
        link_id_[frameName] = link_id;
      } else {
      } // Joint
    }
  }

  assert(joint_id_.size() == n_a_);

  for (size_t i = 1; i < (size_t)(model_.njoints); ++i) {
    total_mass_ += model_.inertias[i].mass();
  }

  joint_pos_limits_.resize(n_a_, 2);
  joint_vel_limits_.resize(n_a_, 2);
  joint_trq_limits_.resize(n_a_, 2);

  if (b_fixed_base_) {
    joint_pos_limits_.block(0, 0, n_a_, 1) = model_.lowerPositionLimit;
    joint_pos_limits_.block(0, 1, n_a_, 1) = model_.upperPositionLimit;
  } else {
    joint_pos_limits_.block(0, 0, n_a_, 1) =
        model_.lowerPositionLimit.segment(n_float_, n_a_);
    joint_pos_limits_.block(0, 1, n_a_, 1) =
        model_.upperPositionLimit.segment(n_float_, n_a_);
    joint_vel_limits_.block(0, 0, n_a_, 1) =
        -model_.velocityLimit.segment(n_float_, n_a_);
    joint_vel_limits_.block(0, 1, n_a_, 1) =
        model_.velocityLimit.segment(n_float_, n_a_);
    joint_trq_limits_.block(0, 0, n_a_, 1) =
        -model_.effortLimit.segment(n_float_, n_a_);
    joint_trq_limits_.block(0, 1, n_a_, 1) =
        model_.effortLimit.segment(n_float_, n_a_);
  }
}

// TODO:
Eigen::Vector3d PinocchioRobotSystem::GetBaseLocalComPos() {}

// TODO:
std::string PinocchioRobotSystem::GetBaseLinkName() {}

int PinocchioRobotSystem::GetQIdx(const std::string &joint_name) {
  return model_.joints[model_.getJointId(joint_name)].idx_q();
}

int PinocchioRobotSystem::GetQdotIdx(const std::string &joint_name) {
  return model_.joints[model_.getJointId(joint_name)].idx_v();
}

int PinocchioRobotSystem::GetJointIdx(const std::string &joint_name) {
  return model_.joints[model_.getJointId(joint_name)].idx_v() - n_float_;
}

// TODO:
std::map<std::string, double>
PinocchioRobotSystem::EigenVectorToMap(const Eigen::VectorXd &cmd_vec) {}

// TODO:
Eigen::VectorXd
PinocchioRobotSystem::MapToEigenVector(std::map<std::string, double> _map) {}

void PinocchioRobotSystem::UpdateRobotModel(
    const Eigen::Vector3d &base_com_pos,
    const Eigen::Quaternion<double> &base_com_quat,
    const Eigen::Vector3d &base_com_lin_vel,
    const Eigen::Vector3d &base_com_ang_vel,
    const Eigen::Vector3d &base_joint_pos,
    const Eigen::Quaternion<double> &base_joint_quat,
    const Eigen::Vector3d &base_joint_lin_vel,
    const Eigen::Vector3d &base_joint_ang_vel,
    const std::map<std::string, double> joint_pos,
    const std::map<std::string, double> joint_vel,
    const bool _b_update_centroid) {

  assert(joint_pos.size() == n_a_);

  Eigen::VectorXd quat_vec(4);
  quat_vec << base_joint_quat.x(), base_joint_quat.y(), base_joint_quat.z(),
      base_joint_quat.w();

  if (!b_fixed_base_) {

    q_.segment(0, 3) = base_joint_pos;
    q_.segment(3, 4) = quat_vec;

    Eigen::Matrix<double, 3, 3> rot_w_basejoint =
        base_joint_quat.normalized().toRotationMatrix();
    Eigen::Matrix<double, 6, 1> twist_basejoint_in_world;
    twist_basejoint_in_world.segment(0, 3) = base_joint_ang_vel;
    twist_basejoint_in_world.segment(3, 3) = base_joint_lin_vel;

    Eigen::Matrix<double, 6, 6> augrot_joint_world;
    augrot_joint_world.setZero();
    augrot_joint_world.block(0, 0, 3, 3) = rot_w_basejoint.transpose();
    augrot_joint_world.block(3, 3, 3, 3) = rot_w_basejoint.transpose();

    Eigen::Matrix<double, 6, 1> twist_basejoint_in_joint =
        augrot_joint_world * twist_basejoint_in_world;
    qdot_.segment(0, 3) = twist_basejoint_in_joint.segment(3, 3);
    qdot_.segment(3, 3) = twist_basejoint_in_joint.segment(0, 3);
  } else {
    // fixed base robot
  }

  for (const auto &kv : joint_pos) {
    q_(GetQIdx(kv.first)) = kv.second;
    joint_positions_(GetJointIdx(kv.first)) = kv.second;
  }
  for (const auto &kv : joint_vel) {
    qdot_(GetQdotIdx(kv.first)) = kv.second;
    joint_velocities_(GetJointIdx(kv.first)) = kv.second;
  }
  // for (const auto & [key, value] : joint_pos) {
  // q_(GetQIdx(key)) = value;
  // joint_positions_(GetJointIdx(key)) = value;
  //}
  // for (const auto & [key, value] : joint_vel) {
  // qdot_(GetQdotIdx(key)) = value;
  // joint_velocities_(GetJointIdx(key)) = value;
  //}

  forwardKinematics(model_, data_, q_, qdot_);

  if (_b_update_centroid) {
    UpdateCentroidalQuantities();
  }
}

void PinocchioRobotSystem::UpdateCentroidalQuantities() {
  pinocchio::ccrba(model_, data_, q_, qdot_);

  Hg_.segment(0, 3) = data_.hg.angular();
  Hg_.segment(3, 3) = data_.hg.linear();

  Ag_.topRows(3) = data_.Ag.template middleRows<3>(pinocchio::Force::ANGULAR);
  Ag_.bottomRows(3) = data_.Ag.template middleRows<3>(pinocchio::Force::LINEAR);

  Ig_.setZero();
  Ig_.block(0, 0, 3, 3) = data_.Ig.matrix().block(3, 3, 3, 3);
  Ig_.block(3, 3, 3, 3) = data_.Ig.matrix().block(0, 0, 3, 3);
}

Eigen::VectorXd PinocchioRobotSystem::GetQ() const { return this->q_; }

Eigen::VectorXd PinocchioRobotSystem::GetQdot() const { return this->qdot_; }

Eigen::MatrixXd PinocchioRobotSystem::GetMassMatrix() {
  crba(model_, data_, q_);
  data_.M.triangularView<Eigen::StrictlyLower>() =
      data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  return data_.M;
}

Eigen::VectorXd PinocchioRobotSystem::GetGravity() {
  return computeGeneralizedGravity(model_, data_, q_);
}

Eigen::VectorXd PinocchioRobotSystem::GetCoriolis() {
  return nonLinearEffects(model_, data_, q_, qdot_) -
         computeGeneralizedGravity(model_, data_, q_);
}

Eigen::Vector3d PinocchioRobotSystem::GetRobotComPos() {
  crba(model_, data_, q_);
  return centerOfMass(model_, data_, q_, qdot_);
}

Eigen::Vector3d PinocchioRobotSystem::GetRobotComLinVel() {
  crba(model_, data_, q_);
  centerOfMass(model_, data_, q_, qdot_);
  return data_.vcom[0];
}

Eigen::Matrix<double, 3, Eigen::Dynamic>
PinocchioRobotSystem::GetComLinJacobian() {
  crba(model_, data_, q_);
  return jacobianCenterOfMass(model_, data_, q_);
  ;
}

Eigen::Matrix<double, 3, Eigen::Dynamic>
PinocchioRobotSystem::GetComLinJacobianDot() {
  return (computeCentroidalMapTimeVariation(model_, data_, q_, qdot_)
              .topRows(3)) /
         total_mass_;
}

Eigen::Isometry3d
PinocchioRobotSystem::GetLinkIso(const std::string &link_name) {
  Eigen::Isometry3d ret;
  pinocchio::Model::Index frame_id = model_.getFrameId(link_name);
  const pinocchio::SE3Tpl<double, 0> trans =
      updateFramePlacement(model_, data_, frame_id);
  ret.linear() = trans.rotation();
  ret.translation() = trans.translation();
  return ret;
}

Eigen::Matrix<double, 6, 1>
PinocchioRobotSystem::GetLinkVel(const std::string &link_name) {
  Eigen::Matrix<double, 6, 1> ret;
  pinocchio::Model::Index frame_id = model_.getFrameId(link_name);
  pinocchio::Motion vf =
      getFrameVelocity(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED);

  ret.segment(0, 3) = vf.angular();
  ret.segment(3, 3) = vf.linear();
  return ret;
}

Eigen::Matrix<double, 6, Eigen::Dynamic>
PinocchioRobotSystem::GetLinkJacobian(const std::string &link_name) {
  pinocchio::Model::Index frame_id = model_.getFrameId(link_name);
  computeJointJacobians(model_, data_, q_);

  Eigen::MatrixXd jac(6, n_qdot_);
  getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED,
                   jac);

  Eigen::MatrixXd ret(6, n_qdot_);
  ret.topRows(3) = jac.bottomRows(3);
  ret.bottomRows(3) = jac.topRows(3);

  return ret;
}

Eigen::Matrix<double, 6, 1>
PinocchioRobotSystem::GetLinkJacobianDotQdot(const std::string &link_name) {
  pinocchio::Model::Index frame_id = model_.getFrameId(link_name);

  forwardKinematics(model_, data_, q_, qdot_, 0 * qdot_);
  pinocchio::Motion af = getFrameClassicalAcceleration(
      model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED);

  Eigen::Matrix<double, 6, 1> ret;
  ret.segment(0, 3) = af.angular();
  ret.segment(3, 3) = af.linear();

  return ret;
}

Eigen::Isometry3d PinocchioRobotSystem::GetLinkIso(const int &link_id) {
  Eigen::Isometry3d ret;
  const pinocchio::SE3Tpl<double, 0> trans =
      updateFramePlacement(model_, data_, link_id);
  ret.linear() = trans.rotation();
  ret.translation() = trans.translation();
  return ret;
}

Eigen::Matrix<double, 6, 1>
PinocchioRobotSystem::GetLinkVel(const int &link_id) {
  Eigen::Matrix<double, 6, 1> ret;
  pinocchio::Motion fv =
      getFrameVelocity(model_, data_, link_id, pinocchio::LOCAL_WORLD_ALIGNED);
  ret.segment(0, 3) = fv.angular();
  ret.segment(3, 3) = fv.linear();
  return ret;
}

Eigen::Matrix<double, 6, Eigen::Dynamic>
PinocchioRobotSystem::GetLinkJacobian(const int &link_id) {
  computeJointJacobians(model_, data_, q_);

  Eigen::MatrixXd jac(6, n_qdot_);
  getFrameJacobian(model_, data_, link_id, pinocchio::LOCAL_WORLD_ALIGNED, jac);
  Eigen::MatrixXd ret(6, n_qdot_);
  ret.topRows(3) = jac.bottomRows(3);
  ret.bottomRows(3) = jac.topRows(3);
  return ret;
}

Eigen::Matrix<double, 6, 1>
PinocchioRobotSystem::GetLinkJacobianDotQdot(const int &link_id) {
  forwardKinematics(model_, data_, q_, qdot_, 0 * qdot_);
  pinocchio::Motion fa = getFrameClassicalAcceleration(
      model_, data_, link_id, pinocchio::LOCAL_WORLD_ALIGNED);

  Eigen::Matrix<double, 6, 1> ret;
  ret.segment(0, 3) = fa.angular();
  ret.segment(3, 3) = fa.linear();

  return ret;
}

void PinocchioRobotSystem::PrintRobotInfo() {
  std::cout << "=======================" << std::endl;
  std::cout << "Pinocchio robot info" << std::endl;
  std::cout << "=======================" << std::endl;

  std::cout << "============ draco link ================" << std::endl;
  for (pinocchio::FrameIndex i = 0;
       i < static_cast<pinocchio::FrameIndex>(model_.nframes); ++i) {
    std::cout << "constexpr int " << model_.frames[i].name << " = "
              << model_.getFrameId(model_.frames[i].name) << ";" << std::endl;
  }
  std::cout << "============ draco joint ================" << std::endl;
  for (pinocchio::JointIndex i = 0;
       i < static_cast<pinocchio::JointIndex>(model_.njoints); ++i) {
    std::cout << "constexpr int " << model_.names[i] << " = "
              << model_.getJointId(model_.names[i]) << ";" << std::endl;
  }

  std::cout << "============ draco ================" << std::endl;
  // std::cout << "constexpr int n_link = "
  //<< static_cast<pinocchio::FrameIndex>(model_.nframes) << ";"
  //<< std::endl;
  std::cout << "constexpr int n_dof = " << qdot_.size() << ";" << std::endl;
  std::cout << "constexpr int n_vdof = " << n_vdof_ << ";" << std::endl;
  std::cout << "constexpr int n_adof = " << qdot_.size() - n_float_ - n_vdof_
            << ";" << std::endl;
  exit(0);
}

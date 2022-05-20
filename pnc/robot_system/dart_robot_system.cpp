#include "pnc/robot_system/dart_robot_system.hpp"
#include "util/util.hpp"

DartRobotSystem::DartRobotSystem(const std::string &_urdf_path,
                                 const bool _b_fixed_base,
                                 const bool _b_print_info, const int _n_vdof)
    : RobotSystem(_b_fixed_base, _b_print_info), n_vdof_(_n_vdof) {
  util::PrettyConstructor(1, "DartRobotSystem");
  dart::utils::DartLoader urdf_loader;
  skeleton_ = urdf_loader.parseSkeleton(_urdf_path);

  this->ConfigRobot();

  joint_positions_.resize(n_a_);
  joint_velocities_.resize(n_a_);

  Ag_.resize(6, n_qdot_);

  if (b_print_info_) {
    this->PrintRobotInfo();
  }
}

DartRobotSystem::DartRobotSystem(dart::dynamics::SkeletonPtr _robot,
                                 const bool _b_fixed_base,
                                 const bool _b_print_info)
    : RobotSystem(_b_fixed_base, _b_print_info) {
  skeleton_ = _robot;

  this->ConfigRobot();

  joint_positions_.resize(n_a_);
  joint_velocities_.resize(n_a_);

  Ag_.resize(6, n_qdot_);
}

void DartRobotSystem::ConfigRobot() {
  if (b_fixed_base_) {
    skeleton_->getRootBodyNode()
        ->changeParentJointType<dart::dynamics::WeldJoint>();
    n_float_ = 0;
  }
  for (int i = 0; i < skeleton_->getNumJoints(); ++i) {
    dart::dynamics::JointPtr joint = skeleton_->getJoint(i);
    if (joint->getName() == "rootJoint") {
      n_float_ = joint->getNumDofs();
    } else if (joint->getType() != "WeldJoint") {
      joint_ptr_map_[joint->getName()] = joint;
    } else {
    }
  }

  for (int i = 0; i < skeleton_->getNumBodyNodes(); ++i) {
    dart::dynamics::BodyNodePtr body_node = skeleton_->getBodyNode(i);
    body_node_ptr_map_[body_node->getName()] = body_node;
  }

  n_q_ = skeleton_->getNumDofs();
  n_qdot_ = skeleton_->getNumDofs();
  n_a_ = n_qdot_ - n_float_;
  total_mass_ = skeleton_->getMass();

  joint_pos_limits_.resize(n_a_, 2);
  joint_vel_limits_.resize(n_a_, 2);
  joint_trq_limits_.resize(n_a_, 2);

  joint_pos_limits_.block(0, 0, n_a_, 1) =
      skeleton_->getPositionLowerLimits().segment(n_float_, n_a_);
  joint_pos_limits_.block(0, 1, n_a_, 1) =
      skeleton_->getPositionUpperLimits().segment(n_float_, n_a_);
  joint_vel_limits_.block(0, 0, n_a_, 1) =
      skeleton_->getVelocityLowerLimits().segment(n_float_, n_a_);
  joint_vel_limits_.block(0, 1, n_a_, 1) =
      skeleton_->getVelocityUpperLimits().segment(n_float_, n_a_);
  joint_trq_limits_.block(0, 0, n_a_, 1) =
      skeleton_->getForceLowerLimits().segment(n_float_, n_a_);
  joint_trq_limits_.block(0, 1, n_a_, 1) =
      skeleton_->getForceUpperLimits().segment(n_float_, n_a_);

  // if (b_print_info_) {
  // std::cout << "===================" << std::endl;
  // std::cout << "Dart Robot Model" << std::endl;
  // std::cout << "n_q:" << n_q_ << std::endl;
  // std::cout << "n_qdot:" << n_qdot_ << std::endl;
  // std::cout << "n_a:" << n_a_ << std::endl;
  // std::cout << "Joint Info: " << std::endl;
  // int count(0);
  // for (std::map<std::string, dart::dynamics::JointPtr>::iterator it =
  // joint_ptr_map_.begin();
  // it != joint_ptr_map_.end(); it++) {
  // std::cout << count << ": " << it->first << std::endl;
  //++count;
  //}

  // std::cout << "Link Info: " << std::endl;
  // count = 0;
  // for (std::map<std::string, dart::dynamics::BodyNodePtr>::iterator it =
  // body_node_ptr_map_.begin();
  // it != body_node_ptr_map_.end(); it++) {
  // std::cout << count << ": " << it->first << std::endl;
  //++count;
  //}

  // std::cout << "Dof Info: " << std::endl;
  // for (int i = 0; i < skeleton_->getNumDofs(); ++i) {
  // dart::dynamics::DegreeOfFreedom *dof = skeleton_->getDof(i);
  // std::cout << i << ": " << dof->getName() << std::endl;
  //}
}
}
int DartRobotSystem::GetQIdx(const std::string &_joint_name) {
  return joint_ptr_map_[_joint_name]->getIndexInSkeleton(0);
}

int DartRobotSystem::GetQdotIdx(const std::string &_joint_name) {
  return joint_ptr_map_[_joint_name]->getIndexInSkeleton(0);
}

int DartRobotSystem::GetJointIdx(const std::string &_joint_name) {
  return joint_ptr_map_[_joint_name]->getIndexInSkeleton(0) - n_float_;
}

std::map<std::string, double>
DartRobotSystem::EigenVectorToMap(const Eigen::VectorXd &_joint_cmd) {
  std::map<std::string, double> ret;
  for (std::map<std::string, dart::dynamics::JointPtr>::iterator it =
           joint_ptr_map_.begin();
       it != joint_ptr_map_.end(); ++it) {
    int idx = this->GetJointIdx(it->first);
    ret[it->first] = _joint_cmd[idx];
  }
  return ret;
}

Eigen::VectorXd
DartRobotSystem::MapToEigenVector(std::map<std::string, double> _joint_map) {
  Eigen::VectorXd vec = Eigen::VectorXd::Zero(_joint_map.size());
  for (std::map<std::string, double>::iterator it = _joint_map.begin();
       it != _joint_map.end(); ++it) {
    int idx = this->GetJointIdx(it->first);
    vec[idx] = it->second;
  }
  return vec;
}

Eigen::Vector3d DartRobotSystem::GetBaseLocalComPos() {
  return skeleton_->getRootBodyNode()->getLocalCOM();
}

std::string DartRobotSystem::GetBaseLinkName() {
  return skeleton_->getRootBodyNode()->getName();
}

void DartRobotSystem::UpdateRobotModel(
    const Eigen::Vector3d &_base_com_pos,
    const Eigen::Quaternion<double> &_base_com_quat,
    const Eigen::Vector3d &_base_com_lin_vel,
    const Eigen::Vector3d &_base_com_ang_vel,
    const Eigen::Vector3d &_base_joint_pos,
    const Eigen::Quaternion<double> &_base_joint_quat,
    const Eigen::Vector3d &_base_joint_lin_vel,
    const Eigen::Vector3d &_base_joint_ang_vel,
    std::map<std::string, double> _joint_positions,
    std::map<std::string, double> _joint_velocities,
    const bool _b_update_centroid) {

  // floating base update
  if (!b_fixed_base_) {

    Eigen::Isometry3d floating_base_joint_iso = Eigen::Isometry3d::Identity();
    floating_base_joint_iso.linear() =
        _base_joint_quat.normalized().toRotationMatrix();
    floating_base_joint_iso.translation() = _base_joint_pos;
    Eigen::Vector6d floating_base_joint_spatial_vel = Eigen::Vector6d::Zero();
    floating_base_joint_spatial_vel.head(3) = _base_joint_ang_vel;
    floating_base_joint_spatial_vel.tail(3) = _base_joint_lin_vel;
    Eigen::Vector6d floating_base_joint_spatial_acc = Eigen::Vector6d::Zero();

    dynamic_cast<dart::dynamics::FreeJoint *>(skeleton_->getRootJoint())
        ->setSpatialMotion(
            &floating_base_joint_iso, dart::dynamics::Frame::World(),
            &floating_base_joint_spatial_vel, dart::dynamics::Frame::World(),
            dart::dynamics::Frame::World(), &floating_base_joint_spatial_acc,
            dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
  } else {
    Eigen::Isometry3d fixed_base_joint_iso = Eigen::Isometry3d::Identity();
    fixed_base_joint_iso.linear() =
        _base_joint_quat.normalized().toRotationMatrix();
    fixed_base_joint_iso.translation() = _base_joint_pos;

    skeleton_->getRootJoint()->setTransformFromParentBodyNode(
        fixed_base_joint_iso);
  }

  // joint pos & vel update
  for (std::map<std::string, double>::iterator it = _joint_positions.begin();
       it != _joint_positions.end(); it++) {
    joint_ptr_map_.find(it->first)->second->setPosition(0, it->second);
    joint_positions_[this->GetJointIdx(it->first)] = it->second;
  }

  for (std::map<std::string, double>::iterator it = _joint_velocities.begin();
       it != _joint_velocities.end(); it++) {
    joint_ptr_map_.find(it->first)->second->setVelocity(0, it->second);
    joint_velocities_[this->GetJointIdx(it->first)] = it->second;
  }

  skeleton_->computeForwardKinematics();
  // skeleton_->computeForwardDynamics(); //no need

  if (_b_update_centroid) {
    this->UpdateCentroidalQuantities();
  }
}

// Update centroid quantities (Ig_, Ag_, Hg_)
void DartRobotSystem::UpdateCentroidalQuantities() {

  Eigen::Matrix<double, 6, Eigen::Dynamic> j_sp;
  Eigen::Vector3d pos_wb; // world to body
  Eigen::Matrix3d rot_wb;
  Eigen::Matrix6d inertia_b;
  Eigen::Isometry3d iso_bc;                        // body to CoM
  Eigen::Vector3d com_pos_w = skeleton_->getCOM(); // CoM wrt World
  Eigen::Matrix6d adjoint_bc;                      // body to CoM
  Ig_.setZero();
  Ag_.setZero();
  Hg_.setZero();

  for (int i = 0; i < skeleton_->getNumBodyNodes(); ++i) {
    dart::dynamics::BodyNodePtr bn = skeleton_->getBodyNode(i);

    j_sp = skeleton_->getJacobian(bn);
    pos_wb = bn->getWorldTransform().translation();
    rot_wb = bn->getWorldTransform().linear();
    inertia_b = bn->getSpatialInertia();
    iso_bc.linear() = rot_wb.transpose();
    iso_bc.translation() = iso_bc.linear() * (com_pos_w - pos_wb);
    adjoint_bc = dart::math::getAdTMatrix(iso_bc);

    Ig_ += adjoint_bc.transpose() * inertia_b * adjoint_bc;
    Ag_ += adjoint_bc.transpose() * inertia_b * j_sp;
  }

  Hg_ = Ag_ * GetQdot();
}

Eigen::VectorXd DartRobotSystem::GetQ() const {
  return skeleton_->getPositions();
}

Eigen::VectorXd DartRobotSystem::GetQdot() const {
  return skeleton_->getVelocities();
}

Eigen::MatrixXd DartRobotSystem::GetMassMatrix() {
  return skeleton_->getMassMatrix();
}

Eigen::VectorXd DartRobotSystem::GetGravity() {
  return skeleton_->getGravityForces();
}

Eigen::VectorXd DartRobotSystem::GetCoriolis() {
  return skeleton_->getCoriolisForces();
}

Eigen::Vector3d DartRobotSystem::GetRobotComPos() {
  return skeleton_->getCOM(dart::dynamics::Frame::World());
}

Eigen::Vector3d DartRobotSystem::GetRobotComLinVel() {
  return skeleton_->getCOMLinearVelocity(dart::dynamics::Frame::World(),
                                         dart::dynamics::Frame::World());
}

Eigen::Isometry3d DartRobotSystem::GetLinkIso(const std::string &_link_name) {
  Eigen::Isometry3d body_node_iso =
      body_node_ptr_map_[_link_name]->getTransform(
          dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
  body_node_iso.translation() =
      body_node_ptr_map_[_link_name]->getCOM(dart::dynamics::Frame::World());

  return body_node_iso;
}

Eigen::Matrix<double, 6, 1>
DartRobotSystem::GetLinkVel(const std::string &_link_name) {
  return body_node_ptr_map_[_link_name]->getCOMSpatialVelocity(
      dart::dynamics::Frame::World(), dart::dynamics::Frame::World());
}

Eigen::Matrix<double, 6, Eigen::Dynamic>
DartRobotSystem::GetLinkJacobian(const std::string &_link_name) {
  return skeleton_->getJacobian(body_node_ptr_map_[_link_name],
                                body_node_ptr_map_[_link_name]->getLocalCOM(),
                                dart::dynamics::Frame::World());
}
Eigen::Matrix<double, 6, 1>
DartRobotSystem::GetLinkJacobianDotQdot(const std::string &_link_name) {
  return skeleton_->getJacobianClassicDeriv(
             body_node_ptr_map_[_link_name],
             body_node_ptr_map_[_link_name]->getLocalCOM(),
             dart::dynamics::Frame::World()) *
         GetQdot();
}

Eigen::Matrix<double, 3, Eigen::Dynamic> DartRobotSystem::GetComLinJacobian() {
  return skeleton_->getCOMLinearJacobian(dart::dynamics::Frame::World());
}

Eigen::Matrix<double, 3, Eigen::Dynamic>
DartRobotSystem::GetComLinJacobianDot() {
  return skeleton_->getCOMLinearJacobianDeriv(dart::dynamics::Frame::World());
}

void DartRobotSystem::PrintRobotInfo() {
  std::cout << "=======================" << std::endl;
  std::cout << "DART robot info" << std::endl;
  std::cout << "=======================" << std::endl;

  std::cout << " ==== draco link ====" << std::endl;
  for (int i = 0; i < skeleton_->getNumBodyNodes(); ++i) {
    dart::dynamics::BodyNodePtr bn = skeleton_->getBodyNode(i);
    std::cout << "constexpr int " << bn->getName() << " = " << std::to_string(i)
              << ";" << std::endl;
  }
  std::cout << " ==== draco joint ====" << std::endl;
  for (int i = 0; i < skeleton_->getNumDofs(); ++i) {
    dart::dynamics::DegreeOfFreedom *dof = skeleton_->getDof(i);
    std::cout << "constexpr int " << dof->getName() << " = "
              << std::to_string(i) << ";" << std::endl;
  }
  std::cout << " ==== draco ====" << std::endl;
  std::cout << "constexpr int n_link = "
            << std::to_string(skeleton_->getNumBodyNodes()) << ";" << std::endl;
  std::cout << "constexpr int n_dof = "
            << std::to_string(skeleton_->getNumDofs()) << ";" << std::endl;
  std::cout << "constexpr int n_vdof = " << std::to_string(n_vdof_) << ";"
            << std::endl;
  std::cout << "constexpr int n_adof = " << std::to_string(n_a_ - n_vdof_)
            << ";" << std::endl;
  exit(0);
}

#include "test/rolling_joint_test_world_node.hpp"
#include "configuration.hpp"
#include "util/util.hpp"

RollingJointWorldNode::RollingJointWorldNode(
    const dart::simulation::WorldPtr &_world)
    : dart::gui::osg::WorldNode(_world), count_(0), t_(0.0), servo_dt_(0) {
  world_ = _world;
  robot_ = world_->getSkeleton("rolling_joint_model");
  n_dof_ = robot_->getNumDofs();

  // robot_model_ = new DartRobotSystem(robot_, true, true);

  // interface_ = new AtlasInterface();
  // sensor_data_ = new AtlasSensorData();
  // command_ = new AtlasCommand();

  // for (int i = 0; i < robot_->getNumJoints(); ++i) {
  // dart::dynamics::Joint *joint = robot_->getJoint(i);
  // if (joint->getNumDofs() == 1) {
  // sensor_data_->joint_positions[joint->getName()] = 0.;
  // sensor_data_->joint_velocities[joint->getName()] = 0.;
  // command_->joint_positions[joint->getName()] = 0.;
  // command_->joint_velocities[joint->getName()] = 0.;
  // command_->joint_torques[joint->getName()] = 0.;
  //}
  //}
  // SetParams();
}

RollingJointWorldNode::~RollingJointWorldNode() {
  // delete robot_model_;
  // delete interface_;
  // delete sensor_data_;
  // delete command_;
}

void RollingJointWorldNode::customPreStep() {
  t_ = (double)count_ * servo_dt_;

  // std::cout << "==============" << std::endl;
  // std::cout << robot_model_->GetLinkIso("end_effector").translation() <<
  // std::endl; std::cout << robot_model_->GetLinkIso("end_effector").linear()
  // << std::endl;

  // Fill Sensor Data
  // GetBaseData_(sensor_data_->base_com_pos, sensor_data_->base_com_quat,
  // sensor_data_->base_com_lin_vel, sensor_data_->base_com_ang_vel,
  // sensor_data_->base_joint_pos, sensor_data_->base_joint_quat,
  // sensor_data_->base_joint_lin_vel,
  // sensor_data_->base_joint_ang_vel);
  // GetJointData_(sensor_data_->joint_positions,
  // sensor_data_->joint_velocities);
  // GetContactSwitchData_(sensor_data_->b_rf_contact,
  // sensor_data_->b_lf_contact);

  // interface_->getCommand(sensor_data_, command_);

  // for (int i = 0; i < robot_->getNumJoints(); ++i) {
  // dart::dynamics::Joint *joint = robot_->getJoint(i);
  // if (joint->getNumDofs() == 1) {
  // double frc = command_->joint_torques[joint->getName()] +
  // kp_ * (command_->joint_positions[joint->getName()] -
  // sensor_data_->joint_positions[joint->getName()]) +
  // kd_ * (command_->joint_velocities[joint->getName()] -
  // sensor_data_->joint_velocities[joint->getName()]);
  // trq_cmd[joint->getIndexInSkeleton(0)] = frc;
  //}
  //}
  // robot_->setForces(trq_cmd);

  count_++;
}

// void RollingJointWorldNode::GetContactSwitchData(bool &rfoot_contact,
// bool &lfoot_contact) {

// double lfoot_height =
// robot_->getBodyNode("l_sole")->getTransform().translation()[2];
// double rfoot_height =
// robot_->getBodyNode("r_sole")->getTransform().translation()[2];

// if (lfoot_height < 0.005) {
// lfoot_contact = true;
//} else {
// lfoot_contact = false;
//}
// if (rfoot_height < 0.005) {
// rfoot_contact = true;
//} else {
// rfoot_contact = false;
//}
//}

// void RollingJointWorldNode::SetParams() {
// try {
// YAML::Node simulation_cfg =
// YAML::LoadFile(THIS_COM "config/atlas/dart_simulation.yaml");
// util::ReadParameter(simulation_cfg, "servo_dt", servo_dt_);
// util::ReadParameter(simulation_cfg["control_configuration"], "kp", kp_);
// util::ReadParameter(simulation_cfg["control_configuration"], "kd", kd_);
//} catch (std::runtime_error &e) {
// std::cout << "Error reading parameter [" << e.what() << "] at file: ["
//<< __FILE__ << "]" << std::endl
//<< std::endl;
//}
//}

// void RollingJointWorldNode::GetBaseData(Eigen::Vector3d &_base_com_pos,
// Eigen::Vector4d &_base_com_quat,
// Eigen::Vector3d &_base_com_lin_vel,
// Eigen::Vector3d &_base_com_ang_vel,
// Eigen::Vector3d &_base_joint_pos,
// Eigen::Vector4d &_base_joint_quat,
// Eigen::Vector3d &_base_joint_lin_vel,
// Eigen::Vector3d &_base_joint_ang_vel) {

// dart::dynamics::BodyNode *root_bn = robot_->getRootBodyNode();

//_base_com_pos = root_bn->getCOM();
// Eigen::Quaternion<double> base_com_quat =
// Eigen::Quaternion<double>(root_bn->getWorldTransform().linear());
//_base_com_quat << base_com_quat.w(), base_com_quat.x(), base_com_quat.y(),
// base_com_quat.z();

//_base_com_ang_vel =
// root_bn->getSpatialVelocity(root_bn->getLocalCOM()).head(3);
//_base_com_lin_vel =
// root_bn->getSpatialVelocity(root_bn->getLocalCOM()).tail(3);

//_base_joint_pos = root_bn->getWorldTransform().translation();
//_base_joint_quat = _base_com_quat;
//_base_joint_ang_vel = root_bn->getSpatialVelocity().head(3);
//_base_joint_lin_vel = root_bn->getSpatialVelocity().tail(3);
//}

// void RollingJointWorldNode::GetJointData(
// std::map<std::string, double> &_joint_positions,
// std::map<std::string, double> &_joint_velocities) {
// for (int i = 0; i < robot_->getNumJoints(); ++i) {
// dart::dynamics::Joint *joint = robot_->getJoint(i);
// if (joint->getNumDofs() == 1) {
// sensor_data_->joint_positions[joint->getName()] = joint->getPosition(0);
// sensor_data_->joint_velocities[joint->getName()] = joint->getVelocity(0);
//}
//}
//}

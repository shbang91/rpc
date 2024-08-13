#include "controller/whole_body_controller/basic_task.hpp"

JointTask::JointTask(PinocchioRobotSystem *robot)
    : Task(robot, robot->NumActiveDof()) {
  util::PrettyConstructor(3, "JointTask");
}

// total active joint task
void JointTask::UpdateOpCommand(const Eigen::Matrix3d &world_R_local) {
  local_des_pos_ = des_pos_;
  pos_ = robot_->GetJointPos();
  local_pos_ = pos_;
  pos_err_ = des_pos_ - pos_;
  local_pos_err_ = pos_err_;

  local_des_vel_ = des_vel_;
  vel_ = robot_->GetJointVel();
  local_vel_ = vel_;
  vel_err_ = des_vel_ - vel_;
  local_vel_err_ = vel_err_;

  local_des_acc_ = des_acc_;

  op_cmd_ = des_acc_ + kp_.cwiseProduct(pos_err_) + kd_.cwiseProduct(vel_err_);
}

void JointTask::UpdateJacobian() {
  jacobian_.block(0, robot_->NumFloatDof(), dim_, robot_->NumActiveDof()) =
      Eigen::MatrixXd::Identity(robot_->NumActiveDof(), robot_->NumActiveDof());
}

void JointTask::UpdateJacobianDotQdot() {
  jacobian_dot_q_dot_ = Eigen::VectorXd::Zero(dim_);
}

// Selected Joint Task
SelectedJointTask::SelectedJointTask(
    PinocchioRobotSystem *robot, const std::vector<int> &joint_idx_container)
    : Task(robot, joint_idx_container.size()) {
  util::PrettyConstructor(3, "SelectedJointTask");
  joint_idx_container_ = joint_idx_container;
}

void SelectedJointTask::UpdateOpCommand(const Eigen::Matrix3d &world_R_local) {
  for (int i = 0; i < dim_; ++i) {
    pos_[i] = robot_->GetQ()[robot_->GetQIdx(joint_idx_container_[i])];
    pos_err_[i] = des_pos_[i] - pos_[i];

    vel_[i] = robot_->GetQdot()[robot_->GetQdotIdx(joint_idx_container_[i])];
    vel_err_[i] = des_vel_[i] - vel_[i];

    op_cmd_[i] = des_acc_[i] + kp_[i] * pos_err_[i] + kd_[i] * vel_err_[i];
  }
  local_des_pos_ = des_pos_;
  local_des_vel_ = des_vel_;
  local_des_acc_ = des_acc_;

  local_pos_ = pos_;
  local_vel_ = vel_;
  local_pos_err_ = pos_err_;
  local_vel_err_ = vel_err_;
}

void SelectedJointTask::UpdateJacobian() {
  for (int i = 0; i < dim_; ++i) {
    int idx = robot_->GetQdotIdx(joint_idx_container_[i]);
    jacobian_(i, idx) = 1.;
  }
}

void SelectedJointTask::UpdateJacobianDotQdot() {
  jacobian_dot_q_dot_ = Eigen::VectorXd::Zero(dim_);
}

// Link Position Task
LinkPosTask::LinkPosTask(PinocchioRobotSystem *robot, int target_idx)
    : Task(robot, 3) {
  util::PrettyConstructor(3, "LinkPosTask");

  target_idx_ = target_idx;
}

void LinkPosTask::UpdateOpCommand(const Eigen::Matrix3d &world_R_local) {
  pos_ = robot_->GetLinkIsometry(target_idx_).translation();
  pos_err_ = des_pos_ - pos_;

  vel_ = robot_->GetLinkSpatialVel(target_idx_).tail(dim_);
  vel_err_ = des_vel_ - vel_;

  // local task data
  local_des_pos_ = world_R_local.transpose() * des_pos_;
  local_pos_ = world_R_local.transpose() * pos_;
  local_pos_err_ = world_R_local.transpose() * pos_err_;

  local_des_vel_ = world_R_local.transpose() * des_vel_;
  local_vel_ = world_R_local.transpose() * vel_;
  local_vel_err_ = world_R_local.transpose() * vel_err_;

  local_des_acc_ = world_R_local.transpose() * des_acc_;

  // operational space command
  op_cmd_ = des_acc_ + world_R_local * (kp_.cwiseProduct(local_pos_err_) +
                                        kd_.cwiseProduct(local_vel_err_));
}

void LinkPosTask::UpdateJacobian() {
  jacobian_ = robot_->GetLinkJacobian(target_idx_)
                  .block(dim_, 0, dim_, robot_->NumQdot());
}

void LinkPosTask::UpdateJacobianDotQdot() {
  jacobian_dot_q_dot_ = robot_->GetLinkJacobianDotQdot(target_idx_).tail(dim_);
}

// Link Orientation Task
LinkOriTask::LinkOriTask(PinocchioRobotSystem *robot, int target_idx)
    : Task(robot, 3) {
  util::PrettyConstructor(3, "LinkOriTask");
  target_idx_ = target_idx;
  des_pos_.resize(4); // quaternion
  des_pos_.setZero();
  pos_.resize(4); // quaternion
  pos_.setZero();
  local_des_pos_.resize(4); // quaternion
  local_des_pos_.setZero();
  local_pos_.resize(4); // quaternion
  local_pos_.setZero();
  des_quat_prev_.setIdentity();
}

void LinkOriTask::UpdateOpCommand(const Eigen::Matrix3d &world_R_local) {
  // save current link orientation
  local_R_world_ = robot_->GetLinkIsometry(target_idx_).linear().transpose();

  Eigen::Quaterniond des_quat(des_pos_[3], des_pos_[0], des_pos_[1],
                              des_pos_[2]);

  Eigen::Quaterniond local_des_quat(world_R_local.transpose() *
                                    des_quat.toRotationMatrix());
  local_des_pos_ << local_des_quat.normalized().coeffs();

  Eigen::Quaterniond quat(robot_->GetLinkIsometry(target_idx_).linear());

  // check desired quat
  // util::AvoidQuatJump(des_quat_prev_, des_quat);

  // if (des_quat.w() < 0.0)
  // des_quat.w() *= -1.0;

  // check actual quat
  util::AvoidQuatJump(des_quat, quat);

  des_quat_prev_ = des_quat; // save prev quat

  pos_ << quat.normalized().coeffs();

  Eigen::Quaterniond local_quat(world_R_local.transpose() *
                                quat.toRotationMatrix());

  local_pos_ << local_quat.normalized().coeffs();

  Eigen::Quaterniond quat_err = des_quat * quat.inverse();

  // if (quat_err.w() < 0.0) {
  // quat_err.w() *= -1.0;
  // quat_err.x() *= -1.0;
  // quat_err.y() *= -1.0;
  // quat_err.z() *= -1.0;
  //}
  // std::cout << "================================" << std::endl;
  // std::cout << "quat err scalar: " << quat_err.w() << std::endl;

  // Eigen::Vector3d so3 = util::QuatToExp(quat_err);
  Eigen::Vector3d so3 = Eigen::AngleAxisd(quat_err).axis();
  so3 *= Eigen::AngleAxisd(quat_err).angle();
  for (int i = 0; i < 3; ++i) {
    pos_err_[i] = so3[i];
  }

  vel_ = robot_->GetLinkSpatialVel(target_idx_).head(dim_);
  vel_err_ = des_vel_ - vel_;

  // local task data
  local_pos_err_ = world_R_local.transpose() * pos_err_;

  local_des_vel_ = world_R_local.transpose() * des_vel_;
  local_vel_ = world_R_local.transpose() * vel_;
  local_vel_err_ = world_R_local.transpose() * vel_err_;

  local_des_acc_ = world_R_local.transpose() * des_acc_;

  // operational space command
  op_cmd_ = des_acc_ + world_R_local * (kp_.cwiseProduct(local_pos_err_) +
                                        kd_.cwiseProduct(local_vel_err_));
}

void LinkOriTask::UpdateJacobian() {
  jacobian_ =
      robot_->GetLinkJacobian(target_idx_).block(0, 0, dim_, robot_->NumQdot());
}

void LinkOriTask::UpdateJacobianDotQdot() {
  jacobian_dot_q_dot_ = robot_->GetLinkJacobianDotQdot(target_idx_).head(dim_);
}

// Robot Center of Mass Task
ComTask::ComTask(PinocchioRobotSystem *robot) : Task(robot, 3) {
  util::PrettyConstructor(3, "Com Task");
  // no target idx
}

void ComTask::UpdateOpCommand(const Eigen::Matrix3d &world_R_local) {
  Eigen::Vector3d com_pos = robot_->GetRobotComPos();
  Eigen::Vector3d com_vel = robot_->GetRobotComLinVel();

  local_des_pos_ = world_R_local.transpose() * des_pos_;
  local_des_vel_ = world_R_local.transpose() * des_vel_;
  local_des_acc_ = world_R_local.transpose() * des_acc_;

  pos_ << com_pos[0], com_pos[1], com_pos[2];
  pos_err_ = des_pos_ - pos_;
  local_pos_ = world_R_local.transpose() * pos_;
  local_pos_err_ = world_R_local.transpose() * pos_err_;

  vel_ << com_vel[0], com_vel[1], com_vel[2];
  vel_err_ = des_vel_ - vel_;
  local_vel_ = world_R_local.transpose() * vel_;
  local_vel_err_ = world_R_local.transpose() * vel_err_;

  op_cmd_ = des_acc_ + world_R_local * (kp_.cwiseProduct(local_pos_err_) +
                                        kd_.cwiseProduct(local_vel_err_));
}

void ComTask::UpdateJacobian() { jacobian_ = robot_->GetComLinJacobian(); }

void ComTask::UpdateJacobianDotQdot() {
  jacobian_dot_q_dot_ = robot_->GetComLinJacobianDotQdot();
}

#include "controller/whole_body_controller/basic_task.hpp"
#include "util/util.hpp"

JointTask::JointTask(PinocchioRobotSystem *robot)
    : Task(robot, robot->GetNumActiveDof()) {
  util::PrettyConstructor(3, "JointTask");
}

// not being used
void JointTask::UpdateOscCommand() {
  pos_ = robot_->GetQ().tail(robot_->GetNumActiveDof());
  pos_err_ = des_pos_ - pos_;

  vel_ = robot_->GetQdot().tail(robot_->GetNumActiveDof());
  vel_err_ = des_vel_ - vel_;

  osc_cmd_ = des_acc_ + kp_.cwiseProduct(pos_err_) + kd_.cwiseProduct(vel_err_);
}

void JointTask::UpdateTaskJacobian() {
  jacobian_.block(0, robot_->GetNumFloatDof(), dim_,
                  robot_->GetNumActiveDof()) =
      Eigen::MatrixXd::Identity(robot_->GetNumActiveDof(),
                                robot_->GetNumActiveDof());
}

void JointTask::UpdateTaskJacobianDotQdot() {
  jacobian_dot_q_dot_ = Eigen::VectorXd::Zero(dim_);
}

// Selected Joint Task
SelectedJointTask::SelectedJointTask(
    PinocchioRobotSystem *robot, const std::vector<int> &joint_idx_container)
    : Task(robot, joint_idx_container.size()) {
  joint_idx_container_ = joint_idx_container;
}

void SelectedJointTask::UpdateOscCommand() {
  for (int i = 0; i < dim_; ++i) {
    pos_[i] = robot_->GetQ()(robot_->GetQIdx(joint_idx_container_[i]));
    pos_err_[i] = des_pos_[i] - pos_[i];

    vel_[i] = robot_->GetQdot()(robot_->GetQdotIdx(joint_idx_container_[i]));
    vel_err_[i] = des_vel_[i] - vel_[i];

    osc_cmd_[i] = des_acc_[i] + kp_[i] * pos_err_[i] + kd_[i] * vel_err_[i];
  }
}

void SelectedJointTask::UpdateTaskJacobian() {
  for (int i = 0; i < dim_; ++i) {
    int idx = robot_->GetQdotIdx(joint_idx_container_[i]);
    jacobian_(i, idx) = 1.;
  }
}

void SelectedJointTask::UpdateTaskJacobianDotQdot() {
  jacobian_dot_q_dot_ = Eigen::VectorXd::Zero(dim_);
}

// Link Position Task
LinkPosTask::LinkPosTask(PinocchioRobotSystem *robot,
                         const int &target_link_idx)
    : Task(robot, 3) {

  target_link_idx_ = target_link_idx;
}

void LinkPosTask::UpdateOscCommand() {
  pos_ = robot_->GetLinkIsometry(target_link_idx_).translation();
  pos_err_ = des_pos_ - pos_;

  vel_ = robot_->GetLinkSpatialVel(target_link_idx_).tail(dim_);
  vel_err_ = des_vel_ - vel_;

  osc_cmd_ = des_acc_ + kp_.cwiseProduct(pos_err_) + kd_.cwiseProduct(vel_err_);
}

void LinkPosTask::UpdateTaskJacobian() {
  jacobian_ = robot_->GetLinkJacobian(target_link_idx_)
                  .block(dim_, 0, dim_, robot_->GetNumQdot());
}

void LinkPosTask::UpdateTaskJacobianDotQdot() {
  jacobian_dot_q_dot_ =
      robot_->GetLinkJacobianDotQdot(target_link_idx_).tail(dim_);
}

// Link Orientation Task
LinkOriTask::LinkOriTask(PinocchioRobotSystem *robot,
                         const int &target_link_idx)
    : Task(robot, 3) {
  target_link_idx_ = target_link_idx;
}

void LinkOriTask::UpdateOscCommand() {
  Eigen::Quaternion<double> quat(
      robot_->GetLinkIsometry(target_link_idx_).linear());
  Eigen::Quaternion<double> des_quat(des_pos_[0], des_pos_[1], des_pos_[2],
                                     des_pos_[3]);

  util::AvoidQuatJump(des_quat, quat);

  Eigen::Quaternion<double> quat_err = des_quat * quat.inverse();

  pos_ << quat.w(), quat.x(), quat.y(), quat.z();
  Eigen::Vector3d so3 = util::QuatToExp(quat_err);
  for (int i = 0; i < 3; ++i) {
    pos_err_[i] = so3[i];
  }

  vel_ = robot_->GetLinkSpatialVel(target_link_idx_).head(dim_);
  vel_err_ = des_vel_ - vel_;

  osc_cmd_ = des_acc_ = kp_.cwiseProduct(pos_err_) + kd_.cwiseProduct(vel_err_);
}

void LinkOriTask::UpdateTaskJacobian() {
  jacobian_ = robot_->GetLinkJacobian(target_link_idx_)
                  .block(0, 0, dim_, robot_->GetNumQdot());
}

void LinkOriTask::UpdateTaskJacobianDotQdot() {
  jacobian_dot_q_dot_ =
      robot_->GetLinkJacobianDotQdot(target_link_idx_).head(dim_);
}

// Robot Center of Mass Task
ComTask::ComTask(PinocchioRobotSystem *robot) : Task(robot, 3) {}

void ComTask::UpdateOscCommand() {
  Eigen::Vector3d com_pos = robot_->GetRobotComPos();
  Eigen::Vector3d com_vel = robot_->GetRobotComLinVel();

  pos_ << com_pos[0], com_pos[1], com_pos[2];
  pos_err_ = des_pos_ - pos_;

  vel_ << com_vel[0], com_vel[1], com_vel[2];
  vel_err_ = des_vel_ - vel_;

  osc_cmd_ = des_acc_ + kp_.cwiseProduct(pos_err_) + kd_.cwiseProduct(vel_err_);
}

void ComTask::UpdateTaskJacobian() { jacobian_ = robot_->GetComLinJacobian(); }

void ComTask::UpdateTaskJacobianDotQdot() {
  jacobian_dot_q_dot_ = robot_->GetComLinJacobianDotQdot();
}

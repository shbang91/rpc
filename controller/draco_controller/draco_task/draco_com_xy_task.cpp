#include "controller/draco_controller/draco_task/draco_com_xy_task.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/pinocchio_robot_system.hpp"

#include <stdexcept>

DracoCoMXYTask::DracoCoMXYTask(PinocchioRobotSystem *robot)
    : Task(robot, 2), b_sim_(false),
      feedback_source_(feedback_source::kCoMFeedback) {
  util::PrettyConstructor(3, "DracoCoMXYTask");

  sp_ = DracoStateProvider::GetStateProvider();
}

void DracoCoMXYTask::UpdateOpCommand() {
  Eigen::Vector2d com_xy_pos = robot_->GetRobotComPos().head<2>();
  Eigen::Vector2d com_xy_vel = b_sim_ ? robot_->GetRobotComLinVel().head<2>()
                                      : sp_->com_vel_est_.head<2>();

  if (feedback_source_ == feedback_source::kCoMFeedback) {
    pos_ << com_xy_pos[0], com_xy_pos[1];
    vel_ << com_xy_vel[0], com_xy_vel[1];

    pos_err_ = des_pos_ - pos_;
    vel_err_ = des_vel_ - vel_;

    op_cmd_ = des_acc_ + kp_.dot(pos_err_) + kd_.dot(vel_err_);

  } else if (feeback_source_ == feedback_source::kIcpFeedback) {
    double omega =
        kGravAcc / sp_->des_com_height_; // TODO: make sure com height task is
                                         // set correctly

    Eigen::Vector2d des_icp = des_pos_ + des_vel_ / omega;
    Eigen::Vector2d des_icp_dot = des_vel_ + des_acc_ / omega;

    // TODO: add integral feedback ctrl law
    Eigen::Vector2d des_cmp = sp_->dcm_.head<2>() - des_icp_dot / omgega +
                              kp_.dot(sp_->dcm_.head<2>() - des_icp);

    op_cmd_ = omega * omega * (com_xy_pos - des_cmp);
  }
}

void DracoCoMXYTask::UpdateJacobian() {
  jacobian_ = robot_->GetComLinJacobian().topRows<2>();
}

void DracoCoMXYTask::UpdateJacobianDotQdot() {
  jacobian_dot_q_dot_ = robot_->GetComLinJacobianDotQdot().head<2>();
}

void DracoCoMXYTask::SetParameters(const YAML::Node node, const bool b_sim) {
  try {

    b_sim_ = b_sim;

    util::ReadParameter(node, "com_feedback_source", feedback_source_);

    std::string prefix = b_sim ? "sim" : "exp";
    if (feedback_source_ == feedback_source::kCoMFeedback) {
      util::ReadParameter(node, prefix + "_kp", kp_);
      util::ReadParameter(node, prefix + "_kd", kd_);
      util::ReadParameter(node, prefix + "_weight", weight_);
    } else if (feedback_source_ == feedback_source::kIcpFeedback) {
      util::ReadParameter(node, prefix + "_icp_kp", kp_);
      util::ReadParameter(node, prefix + "_icp_kd", kd_);
      util::ReadParameter(node, prefix + "_icp_ki", ki_);
      util::ReadParameter(node, prefix + "_icp_weight", weight_);
    } else
      throw std::invalid_argument("No Matching CoM Feedback Source");

  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  } catch (const std::invalid_argument &ex) {
    std::cerr << "Error: " << ex.what() << " at file: [" << __FILE__ << "]"
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

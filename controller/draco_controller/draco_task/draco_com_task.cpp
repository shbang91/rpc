#include "controller/draco_controller/draco_task/draco_com_task.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"

DracoComTask::DracoComTask(PinocchioRobotSystem *robot)
    : Task(robot, 3), com_feedback_source_(com_feedback_source::kComFeedback),
      com_height_target_source_(com_height_target_source::kComHeight) {
  util::PrettyConstructor(3, "DracoCoMTask");

  sp_ = DracoStateProvider::GetStateProvider();
}

void DracoComTask::UpdateOscCommand() {
  if (com_feedback_source_ == com_feedback_source::kComFeedback) {
    Eigen::Vector3d com_pos = robot_->GetRobotComPos();
    Eigen::Vector3d com_vel = robot_->GetRobotComLinVel();

    pos_ << com_pos[0], com_pos[1], com_pos[2];
    vel_ << com_vel[0], com_vel[1], com_vel[2];

    if (com_height_target_source_ == com_height_target_source::kComHeight) {
      pos_[2] = robot_->GetRobotComPos()[2];
      vel_[2] = robot_->GetRobotComLinVel()[2];
    } else if (com_height_target_source_ ==
               com_height_target_source::kBaseHeight) {
      pos_[2] =
          robot_->GetLinkIsometry(draco_link::torso_com_link).translation()[2];
      vel_[2] = robot_->GetLinkSpatialVel(draco_link::torso_com_link)[5];
    } else {
      throw std::invalid_argument("No Matching CoM Height Task Target");
    }

    pos_err_ = des_pos_ - pos_;
    vel_err_ = des_vel_ - vel_;

    osc_cmd_ =
        des_acc_ + kp_.cwiseProduct(pos_err_) + kd_.cwiseProduct(vel_err_);

  } else if (com_feedback_source_ == com_feedback_source::kDcmFeedback) {
    Eigen::Vector3d com_pos = robot_->GetRobotComPos();
    Eigen::Vector3d com_vel = robot_->GetRobotComLinVel();

    double omega = sqrt(9.81 / des_pos_[2]); // TODO: base height / com height?
    Eigen::Vector2d des_dcm = des_pos_.head(2) + des_vel_.head(2) / omega;
    Eigen::Vector2d des_dcm_dot = des_vel_.head(2) + des_acc_.head(2) / omega;

    if (com_height_target_source_ == com_height_target_source::kComHeight) {
      pos_[2] = robot_->GetRobotComPos()[2];
      vel_[2] = robot_->GetRobotComLinVel()[2];
    } else if (com_height_target_source_ ==
               com_height_target_source::kBaseHeight) {
      pos_[2] =
          robot_->GetLinkIsometry(draco_link::torso_com_link).translation()[2];
      vel_[2] = robot_->GetLinkSpatialVel(draco_link::torso_com_link)[5];
    } else {
      throw std::invalid_argument("No Matching CoM Height Task Target");
    }

    Eigen::Vector2d des_cmp =
        sp_->dcm_.head(2) + omega * des_dcm_dot +
        kp_.head(2).cwiseProduct(sp_->dcm_.head(2) - des_dcm);

    osc_cmd_.head(2) = omega * omega * (com_pos.head(2) - des_cmp);
    osc_cmd_[2] =
        kp_[2] * (des_pos_[2] - pos_[2]) + kd_[2] * (des_vel_[2] - vel_[2]);
  } else {
    throw std::invalid_argument("No Matching Feedback Source on CoM Task");
  }
}

void DracoComTask::UpdateTaskJacobian() {
  if (com_height_target_source_ == com_height_target_source::kComHeight) {
    jacobian_ = robot_->GetComLinJacobian();
  } else if (com_height_target_source_ ==
             com_height_target_source::kBaseHeight) {
    jacobian_.block(0, 0, 2, robot_->GetNumQdot()) =
        robot_->GetComLinJacobian().block(0, 0, 2, robot_->GetNumQdot());
    jacobian_.block(2, 0, 1, robot_->GetNumQdot()) =
        robot_->GetLinkJacobian(draco_link::torso_com_link)
            .block(5, 0, 1, robot_->GetNumQdot());
  } else {
    throw std::invalid_argument("No Matching CoM Task Jacobian");
  }
}

void DracoComTask::UpdateTaskJacobianDotQdot() {
  if (com_height_target_source_ == com_height_target_source::kComHeight) {
    jacobian_dot_q_dot_ = robot_->GetComLinJacobianDotQdot();
  } else if (com_height_target_source_ ==
             com_height_target_source::kBaseHeight) {
    jacobian_dot_q_dot_.head(2) = robot_->GetComLinJacobianDotQdot().head(2);
    jacobian_dot_q_dot_[2] =
        robot_->GetLinkJacobianDotQdot(draco_link::torso_com_link)[5];
  } else {
    throw std::invalid_argument("No Matching CoM Task JacobianDotQdot");
  }
}

void DracoComTask::SetTaskParameters(const YAML::Node &node,
                                     const bool &b_sim) {
  try {
    util::ReadParameter(node, "com_feedback_source", com_feedback_source_);
    util::ReadParameter(node, "com_height_target_source",
                        com_height_target_source_);
    if (b_sim) {
      if (com_feedback_source_ == com_feedback_source::kComFeedback) {
        util::ReadParameter(node, "kp", kp_);
        util::ReadParameter(node, "kd", kd_);
        util::ReadParameter(node, "weight", task_component_hierarchy_);
      } else if (com_feedback_source_ == com_feedback_source::kDcmFeedback) {
        util::ReadParameter(node, "icp_kp", kp_);
        util::ReadParameter(node, "icp_kd", kd_);
        util::ReadParameter(node, "icp_ki", ki_);
        util::ReadParameter(node, "icp_weight", task_component_hierarchy_);
      } else {
        throw std::invalid_argument("No Matching CoM Feedback Source");
      }
    } else {
      if (com_feedback_source_ == com_feedback_source::kComFeedback) {
        util::ReadParameter(node, "exp_kp", kp_);
        util::ReadParameter(node, "exp_kd", kd_);
        util::ReadParameter(node, "exp_weight", task_component_hierarchy_);
      } else if (com_feedback_source_ == com_feedback_source::kDcmFeedback) {
        util::ReadParameter(node, "exp_icp_kp", kp_);
        util::ReadParameter(node, "exp_icp_kd", kd_);
        util::ReadParameter(node, "exp_icp_ki", ki_);
        util::ReadParameter(node, "exp_icp_weight", task_component_hierarchy_);
      } else {
        throw std::invalid_argument("No Matching CoM Feedback Source");
      }
    }
  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

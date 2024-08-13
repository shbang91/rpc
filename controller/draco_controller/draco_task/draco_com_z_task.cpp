#include "controller/draco_controller/draco_task/draco_com_z_task.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"

#include <stdexcept>

DracoCoMZTask::DracoCoMZTask(PinocchioRobotSystem *robot) : Task(robot, 1) {

  util::PrettyConstructor(3, "DracoCoMZTask");
  sp_ = DracoStateProvider::GetStateProvider();
}

void DracoCoMZTask::UpdateOpCommand(const Eigen::Matrix3d &world_R_local) {
  if (com_height_ == com_height::kCoM) {
    pos_ << robot_->GetRobotComPos()[2];
    vel_ << sp_->com_vel_est_[2];

  } else if (com_height_ == com_height::kBase) {
    pos_
        << robot_->GetLinkIsometry(draco_link::torso_com_link).translation()[2];
    vel_ << robot_->GetLinkSpatialVel(draco_link::torso_com_link)[5];
  }

  pos_err_ = des_pos_ - pos_;
  vel_err_ = des_vel_ - vel_;

  //=============================================================
  // local com z task data
  //=============================================================
  local_des_pos_ = des_pos_;
  local_pos_ = pos_;
  local_pos_err_ = pos_err_;

  local_des_vel_ = des_vel_;
  local_vel_ = vel_;
  local_vel_err_ = vel_err_;

  local_des_acc_ = des_acc_;

  op_cmd_ = des_acc_ + kp_.cwiseProduct(pos_err_) + kd_.cwiseProduct(vel_err_);
}

void DracoCoMZTask::UpdateJacobian() {
  if (com_height_ == com_height::kCoM) {
    jacobian_ = robot_->GetComLinJacobian().bottomRows<1>();
  } else if (com_height_ == com_height::kBase) {
    jacobian_ =
        robot_->GetLinkJacobian(draco_link::torso_com_link).bottomRows<1>();
  }
}

void DracoCoMZTask::UpdateJacobianDotQdot() {
  if (com_height_ == com_height::kCoM) {
    jacobian_dot_q_dot_ << robot_->GetComLinJacobianDotQdot()[2];
  } else if (com_height_ == com_height::kBase) {
    jacobian_dot_q_dot_ << robot_->GetLinkJacobianDotQdot(
        draco_link::torso_com_link)[5];
  }
}

void DracoCoMZTask::SetParameters(const YAML::Node &node,
                                  const WBC_TYPE wbc_type) {
  try {
    util::ReadParameter(node, "com_height_target_source", com_height_);
    sp_->b_use_base_height_ = com_height_ == com_height::kBase ? true : false;

    if (wbc_type == WBC_TYPE::IHWBC) {
      if (com_height_ == com_height::kCoM)
        util::ReadParameter(node, "com_weight", weight_);
      else if (com_height_ == com_height::kBase)
        util::ReadParameter(node, "base_weight", weight_);
    } else if (wbc_type == WBC_TYPE::WBIC)
      util::ReadParameter(node, "kp_ik", kp_ik_);

    if (com_height_ == com_height::kCoM) {
      util::ReadParameter(node, "com_kp", kp_);
      util::ReadParameter(node, "com_kd", kd_);
    } else if (com_height_ == com_height::kBase) {
      util::ReadParameter(node, "base_kp", kp_);
      util::ReadParameter(node, "base_kd", kd_);
    } else
      throw std::invalid_argument("No Matching CoM Height Target Source");

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

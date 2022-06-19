#include "controller/draco_controller/draco_state_machines/double_support_stand_up.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_task/draco_com_task.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "util/util.hpp"

DoubleSupportStandUp::DoubleSupportStandUp(const StateId state_id,
                                           PinocchioRobotSystem *robot,
                                           DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch), duration_(0.),
      target_height_(0.) {
  util::PrettyConstructor(2, "DoubleSupportStandUp");

  sp_ = DracoStateProvider::GetStateProvider();

  // check if desired com height is using base height
  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
  int height_source = util::ReadParameter<int>(cfg["wbc"]["task"]["com_task"],
                                               "com_height_target_source");
  b_use_base_height_ =
      height_source == com_height_target_source::kBaseHeight ? true : false;
}

void DoubleSupportStandUp::FirstVisit() {
  std::cout << "draco_states::kDoubleSupportStandUp" << std::endl;
  state_machine_start_time_ = sp_->current_time_;

  // desired com & orientation setting
  Eigen::Vector3d target_com_pos =
      (robot_->GetLinkIsometry(draco_link::l_foot_contact).translation() +
       robot_->GetLinkIsometry(draco_link::r_foot_contact).translation()) /
      2.;
  target_com_pos[2] = target_height_; // base or com

  Eigen::Quaterniond l_foot_quat(
      robot_->GetLinkIsometry(draco_link::l_foot_contact).linear());
  Eigen::Quaterniond r_foot_quat(
      robot_->GetLinkIsometry(draco_link::r_foot_contact).linear());
  Eigen::Quaterniond foot_interpol_quat = l_foot_quat.slerp(0.5, r_foot_quat);

  // parallel to flat ground
  Eigen::Vector3d rot_z(0, 0, 1);

  Eigen::Vector3d rot_y = foot_interpol_quat.toRotationMatrix().col(1);
  Eigen::Vector3d rot_x = rot_y.cross(rot_z);
  Eigen::Matrix3d target_torso_SO3;
  target_torso_SO3.col(0) = rot_x;
  target_torso_SO3.col(1) = rot_y;
  target_torso_SO3.col(2) = rot_z;
  Eigen::Quaterniond target_torso_quat(target_torso_SO3);

  ctrl_arch_->floating_base_tm_->InitializeFloatingBaseInterpolation(
      target_com_pos, target_torso_quat.normalized(), duration_,
      b_use_base_height_);

  // TODO:
  //  increase maximum normal reaction force
}

void DoubleSupportStandUp::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_time_;

  ctrl_arch_->floating_base_tm_->UpdateDesired(state_machine_time_);

  // TODO:
  // foot task

  // TODO:
  //  increase maximum normal reaction force
}

void DoubleSupportStandUp::LastVisit() {}

bool DoubleSupportStandUp::EndOfState() {
  return (state_machine_time_ > duration_) ? true : false;
}

StateId DoubleSupportStandUp::GetNextState() {
  return draco_states::kDoubleSupportBalance;
}

void DoubleSupportStandUp::InitializeParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "standup_duration", duration_);
    target_height_ =
        (b_use_base_height_)
            ? util::ReadParameter<double>(node, "target_base_height")
            : util::ReadParameter<double>(node, "target_com_height");
  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

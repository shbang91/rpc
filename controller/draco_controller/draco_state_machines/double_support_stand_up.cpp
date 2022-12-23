#include "controller/draco_controller/draco_state_machines/double_support_stand_up.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_task/draco_com_task.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "util/util.hpp"

namespace {
void MakeHorizontal(Eigen::Isometry3d &pose) {
  const Eigen::Matrix3d R = pose.linear();
  const Eigen::Vector3d p = pose.translation();
  Eigen::Vector3d rpy = util::rpyFromRotMat(R);
  pose.translation() = Eigen::Vector3d{p(0), p(1), 0.};
  pose.linear() = util::rpyToRotMat(0., 0., rpy(2));
}

} // namespace

DoubleSupportStandUp::DoubleSupportStandUp(const StateId state_id,
                                           PinocchioRobotSystem *robot,
                                           DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch),
      standup_duration_(0.), target_height_(0.), rf_z_max_interp_duration_(0.) {
  util::PrettyConstructor(2, "DoubleSupportStandUp");

  sp_ = DracoStateProvider::GetStateProvider();
}

void DoubleSupportStandUp::FirstVisit() {
  std::cout << "draco_states::kDoubleSupportStandUp" << std::endl;
  state_machine_start_time_ = sp_->current_time_;

  // initial com & torso ori setting
  Eigen::Vector3d init_com_pos = robot_->GetRobotComPos();
  if (sp_->b_use_base_height_)
    init_com_pos[2] =
        robot_->GetLinkIsometry(draco_link::torso_com_link).translation()[2];
  Eigen::Matrix3d R_w_torso =
      robot_->GetLinkIsometry(draco_link::torso_com_link).linear();
  Eigen::Quaterniond init_torso_quat(R_w_torso);

  // desired com & torso ori setting
  Eigen::Isometry3d lfoot_iso =
      robot_->GetLinkIsometry(draco_link::l_foot_contact);
  Eigen::Isometry3d rfoot_iso =
      robot_->GetLinkIsometry(draco_link::r_foot_contact);
  MakeHorizontal(lfoot_iso);
  MakeHorizontal(rfoot_iso);

  Eigen::Vector3d target_com_pos =
      (lfoot_iso.translation() + rfoot_iso.translation()) / 2.;
  target_com_pos[2] = target_height_;

  Eigen::Quaterniond lfoot_quat(lfoot_iso.linear());
  Eigen::Quaterniond rfoot_quat(rfoot_iso.linear());
  Eigen::Quaterniond target_torso_quat = lfoot_quat.slerp(0.5, rfoot_quat);

  // initialize floating trajectory
  ctrl_arch_->floating_base_tm_->InitializeFloatingBaseInterpolation(
      init_com_pos, target_com_pos, init_torso_quat, target_torso_quat,
      standup_duration_);

  //  increase maximum normal reaction force
  ctrl_arch_->lf_max_normal_froce_tm_->InitializeRampToMax(
      rf_z_max_interp_duration_);
  ctrl_arch_->rf_max_normal_froce_tm_->InitializeRampToMax(
      rf_z_max_interp_duration_);
}

void DoubleSupportStandUp::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // com & torso ori task update
  ctrl_arch_->floating_base_tm_->UpdateDesired(state_machine_time_);

  // foot task
  ctrl_arch_->lf_SE3_tm_->UseCurrent();
  ctrl_arch_->rf_SE3_tm_->UseCurrent();

  //  increase maximum normal reaction force
  ctrl_arch_->lf_max_normal_froce_tm_->UpdateRampToMax(state_machine_time_);
  ctrl_arch_->rf_max_normal_froce_tm_->UpdateRampToMax(state_machine_time_);
}

void DoubleSupportStandUp::LastVisit() {}

bool DoubleSupportStandUp::EndOfState() {
  return (state_machine_time_ > standup_duration_) ? true : false;
}

StateId DoubleSupportStandUp::GetNextState() {
  return draco_states::kDoubleSupportBalance;
}

void DoubleSupportStandUp::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "standup_duration", standup_duration_);
    std::string prefix = sp_->b_use_base_height_ ? "base" : "com";
    util::ReadParameter(node, "target_" + prefix + "_height", target_height_);
    util::ReadParameter(node, "rf_z_max_interp_duration",
                        rf_z_max_interp_duration_);
  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

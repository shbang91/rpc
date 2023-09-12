#include "controller/draco_controller/draco_state_machines/double_support_move.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_task/draco_com_xy_task.hpp"
#include "controller/draco_controller/draco_task/draco_com_z_task.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/end_effector_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/floating_base_trajectory_manager.hpp"
#include "util/util.hpp"

DoubleSupportMove::DoubleSupportMove(const StateId state_id,
                                     PinocchioRobotSystem *robot,
                                     DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch),
      b_static_walking_trigger_(false) {
  util::PrettyConstructor(2, "DoubleSupportMove");

  sp_ = DracoStateProvider::GetStateProvider();
}

void DoubleSupportMove::FirstVisit() {
  state_machine_start_time_ = sp_->current_time_;

  sp_->b_lf_contact_ = true;
  sp_->b_rf_contact_ = true;

  Eigen::Vector3d init_com_pos = Eigen::Vector3d::Zero();
  init_com_pos.head<2>() =
      ctrl_arch_->tci_container_->task_map_["com_xy_task"]->DesiredPos();
  init_com_pos[2] =
      ctrl_arch_->tci_container_->task_map_["com_z_task"]->DesiredPos()[0];
  Eigen::VectorXd init_torso_quat_vec =
      ctrl_arch_->tci_container_->task_map_["torso_ori_task"]
          ->DesiredPos(); // q.x, q.y, q.z, q.w
  Eigen::Quaterniond init_torso_quat(
      init_torso_quat_vec[3], init_torso_quat_vec[0], init_torso_quat_vec[1],
      init_torso_quat_vec[2]);

  if (state_id_ == draco_states::kDoubleSupportMoveCoMLeftFoot) {
    std::cout << "draco_states::kDoubleSupportMoveCoMLeftFoot" << std::endl;
    Eigen::Vector3d target_com_pos =
        robot_->GetLinkIsometry(draco_link::l_foot_contact).translation();
    Eigen::Quaterniond target_torso_quat = init_torso_quat;
    target_com_pos +=
        init_torso_quat.toRotationMatrix() * sp_->com_offset_local_;
    target_com_pos[2] = sp_->des_com_height_;

    // initialize floating trajectory
    ctrl_arch_->floating_base_tm_->InitializeFloatingBaseInterpolation(
        init_com_pos, target_com_pos, init_torso_quat, target_torso_quat,
        end_time_);

  } else if (state_id_ == draco_states::kDoubleSupportMoveCoMRightFoot) {
    std::cout << "draco_states::kDoubleSupportMoveCoMRightFoot" << std::endl;
    Eigen::Vector3d target_com_pos =
        robot_->GetLinkIsometry(draco_link::r_foot_contact).translation();
    Eigen::Quaterniond target_torso_quat = init_torso_quat;
    target_com_pos +=
        init_torso_quat.toRotationMatrix() * sp_->com_offset_local_;
    target_com_pos[2] = sp_->des_com_height_;

    // initialize floating trajectory
    ctrl_arch_->floating_base_tm_->InitializeFloatingBaseInterpolation(
        init_com_pos, target_com_pos, init_torso_quat, target_torso_quat,
        end_time_);

  } else if (state_id_ == draco_states::kDoubleSupportMoveCoMCenter) {
    std::cout << "draco_states::kDoubleSupportMoveCoMCenter" << std::endl;
    Eigen::Vector3d target_com_pos =
        0.5 *
        (robot_->GetLinkIsometry(draco_link::l_foot_contact).translation() +
         robot_->GetLinkIsometry(draco_link::r_foot_contact).translation());
    Eigen::Quaterniond target_torso_quat = init_torso_quat;
    target_com_pos +=
        init_torso_quat.toRotationMatrix() * sp_->com_offset_local_;
    target_com_pos[2] = sp_->des_com_height_;

    // initialize floating trajectory
    ctrl_arch_->floating_base_tm_->InitializeFloatingBaseInterpolation(
        init_com_pos, target_com_pos, init_torso_quat, target_torso_quat,
        end_time_);

  } else {
    assert(false);
  }
}

void DoubleSupportMove::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  // com & torso ori task update
  ctrl_arch_->floating_base_tm_->UpdateDesired(state_machine_time_);

  // foot task
  // ctrl_arch_->lf_SE3_tm_->UseCurrent();
  // ctrl_arch_->rf_SE3_tm_->UseCurrent();
  ctrl_arch_->lf_SE3_tm_->UseNominal(sp_->nominal_left_foot_iso_);
  ctrl_arch_->rf_SE3_tm_->UseNominal(sp_->nominal_right_foot_iso_);
}

void DoubleSupportMove::LastVisit() { b_static_walking_trigger_ = false; }

bool DoubleSupportMove::EndOfState() {
  return (state_machine_time_ > end_time_ && b_static_walking_trigger_) ? true
                                                                        : false;
}

StateId DoubleSupportMove::GetNextState() {
  if (state_id_ == draco_states::kDoubleSupportMoveCoMLeftFoot)
    return draco_states::kRFootLiftingTransition;
  if (state_id_ == draco_states::kDoubleSupportMoveCoMRightFoot)
    return draco_states::kLFootLiftingTransition;
  if (state_id_ == draco_states::kDoubleSupportMoveCoMCenter)
    return draco_states::kDoubleSupportBalance;
}

void DoubleSupportMove::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "moving_duration", end_time_);
  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

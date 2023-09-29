#include "controller/draco_controller/draco_state_machines/locomotion.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_task/draco_com_xy_task.hpp"
#include "controller/draco_controller/draco_task/draco_com_z_task.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "convex_mpc/convex_mpc_locomotion.hpp"

Locomotion::Locomotion(const StateId state_id, PinocchioRobotSystem *robot,
                       DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch) {
  util::PrettyConstructor(2, "Locomotion");

  sp_ = DracoStateProvider::GetStateProvider();

  YAML::Node mpc_cfg =
      YAML::LoadFile(THIS_COM "config/draco/MPC_LOCOMOTION.yaml");

  gait_command_ = std::make_shared<GaitCommand>();

  x_vel_cmd_ = util::ReadParameter<double>(mpc_cfg["gait"], "x_vel_cmd");
  y_vel_cmd_ = util::ReadParameter<double>(mpc_cfg["gait"], "y_vel_cmd");
  yaw_rate_cmd_ = util::ReadParameter<double>(mpc_cfg["gait"], "yaw_rate_cmd");
  // TODO: use this in convex mpc locomotion class
  gait_number_ = util::ReadParameter<int>(mpc_cfg["gait"], "gait_number");
  swing_height_ = util::ReadParameter<double>(mpc_cfg["swing_foot"], "height");
}

void Locomotion::FirstVisit() {
  std::cout << "draco_states: kLocomotion" << std::endl;
  state_machine_start_time_ = sp_->current_time_;

  // TODO: initialize convexMPC
  gait_command_->vel_xy_des[0] = x_vel_cmd_;
  gait_command_->vel_xy_des[1] = y_vel_cmd_;
  gait_command_->yaw_rate = yaw_rate_cmd_;
  ctrl_arch_->convex_mpc_locomotion_->Initialize(*gait_command_,
                                                 sp_->des_body_height_);
  ctrl_arch_->convex_mpc_locomotion_->SetGait(gait_number_);
  ctrl_arch_->convex_mpc_locomotion_->SetSwingHeight(swing_height_);
  ctrl_arch_->convex_mpc_locomotion_->SetHipLocation(
      robot_->GetBaseToFootXYOffset());
}

void Locomotion::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  const auto &mpc_interface = ctrl_arch_->convex_mpc_locomotion_;
  const auto &tci_container = ctrl_arch_->tci_container_;

  // solve convexMPC
  mpc_interface->Solve();

  // update desired task
  // 1. centroidal task
  Eigen::Vector3d zero_vec = Eigen::Vector3d::Zero();
  tci_container->task_map_["com_z_task"]->UpdateDesired(
      mpc_interface->des_body_pos_.tail<1>(),
      mpc_interface->des_body_vel_.tail<1>(), zero_vec.tail<1>());
  tci_container->task_map_["com_xy_task"]->UpdateDesired(
      mpc_interface->des_body_pos_.head<2>(),
      mpc_interface->des_body_vel_.head<2>(), Eigen::Vector2d::Zero());
  Eigen::Quaterniond des_body_quat = util::EulerZYXtoQuat(
      mpc_interface->des_body_rpy_[0], mpc_interface->des_body_rpy_[1],
      mpc_interface->des_body_rpy_[2]);
  Eigen::VectorXd des_body_quat_vec(4);
  des_body_quat_vec << des_body_quat.x(), des_body_quat.y(), des_body_quat.z(),
      des_body_quat.w();
  tci_container->task_map_["torso_ori_task"]->UpdateDesired(
      des_body_quat_vec, mpc_interface->des_body_ang_vel_,
      Eigen::Vector3d::Zero());

  // 2. foot task
  tci_container->task_map_["lf_pos_task"]->UpdateDesired(
      mpc_interface->des_foot_pos_[0], mpc_interface->des_foot_vel_[0],
      mpc_interface->des_foot_acc_[0]);
  tci_container->task_map_["rf_pos_task"]->UpdateDesired(
      mpc_interface->des_foot_pos_[1], mpc_interface->des_foot_vel_[1],
      mpc_interface->des_foot_acc_[1]);

  // 3. reaction force task
  tci_container->force_task_map_["lf_force_task"]->UpdateDesiredToLocal(
      mpc_interface->des_lf_wrench_);
  tci_container->force_task_map_["rf_force_task"]->UpdateDesiredToLocal(
      mpc_interface->des_rf_wrench_);
  // std::cout << "======================================================="
  //<< std::endl;
  // std::cout << "lf wrench: " << mpc_interface->des_lf_wrench_.transpose()
  //<< std::endl;
  // std::cout << "rf wrench: " << mpc_interface->des_rf_wrench_.transpose()
  //<< std::endl;

  // TODO:update contact state for controller
}

bool Locomotion::EndOfState() { return false; }

void Locomotion::LastVisit() {}

StateId Locomotion::GetNextState() {}

void Locomotion::SetParameters(const YAML::Node &node) {
  try {
    // util::ReadParameter(node, "amplitude", amp_);
    // util::ReadParameter(node, "frequency", freq_);
  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [ " << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

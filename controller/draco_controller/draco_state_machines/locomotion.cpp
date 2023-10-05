#include "controller/draco_controller/draco_state_machines/locomotion.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_task/draco_com_xy_task.hpp"
#include "controller/draco_controller/draco_task/draco_com_z_task.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "controller/whole_body_controller/wbic/wbic.hpp"
#include "convex_mpc/convex_mpc_locomotion.hpp"

Locomotion::Locomotion(const StateId state_id, PinocchioRobotSystem *robot,
                       DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch) {
  util::PrettyConstructor(2, "Locomotion");

  sp_ = DracoStateProvider::GetStateProvider();

  // TODO: check this wbc yaml
  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/draco/pnc.yaml");
  util::ReadParameter(cfg["wbc"]["qp"], "W_force_rate_of_change_left_foot",
                      W_force_rate_of_change_left_foot_);
  util::ReadParameter(cfg["wbc"]["qp"], "W_force_rate_of_change_right_foot",
                      W_force_rate_of_change_right_foot_);
  util::ReadParameter(cfg["wbc"]["qp"], "W_delta_rf_left_foot_in_contact_mpc",
                      W_delta_rf_lfoot_);
  util::ReadParameter(cfg["wbc"]["qp"], "W_delta_rf_right_foot_in_contact_mpc",
                      W_delta_rf_rfoot_);
  // mpc yaml
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

  // wbc reaction wrench smoother
  ctrl_arch_->tci_container_->qp_params_->W_force_rate_of_change_
      << W_force_rate_of_change_left_foot_,
      W_force_rate_of_change_right_foot_;

  ctrl_arch_->tci_container_->qp_params_->W_delta_rf_ << W_delta_rf_lfoot_,
      W_delta_rf_rfoot_;

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

  // TODO: should be generated inside of convexmpclocomotion class
  lf_ori_quat_ = Eigen::Quaterniond(
      robot_->GetLinkIsometry(draco_link::l_foot_contact).linear());
  rf_ori_quat_ = Eigen::Quaterniond(
      robot_->GetLinkIsometry(draco_link::r_foot_contact).linear());
}

void Locomotion::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;
  const auto &mpc_interface = ctrl_arch_->convex_mpc_locomotion_;
  const auto &tci_container = ctrl_arch_->tci_container_;

  // solve convexMPC
  mpc_interface->Solve();

  // contact and task clear
  auto &contact_vector = ctrl_arch_->tci_container_->contact_vector_;
  auto &contact_map = ctrl_arch_->tci_container_->contact_map_;
  contact_vector.clear();
  auto &task_vector = ctrl_arch_->tci_container_->task_vector_;
  auto &task_map = ctrl_arch_->tci_container_->task_map_;
  task_vector.clear();

  task_vector.push_back(task_map["com_z_task"]);
  task_vector.push_back(task_map["torso_ori_task"]);
  task_vector.push_back(task_map["com_xy_task"]);
  task_vector.push_back(task_map["upper_body_task"]); // des task set up outside

  // update desired task
  // centroidal task
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

  // TODO:update contact state for controller
  for (int leg = 0; leg < 2; leg++) {
    if (mpc_interface->contact_state_[leg] > 0.0) // in contact
    {
      if (leg == 0) {
        // contact
        contact_vector.push_back(contact_map["lf_contact"]);
        sp_->b_lf_contact_ =
            true; // TODO(change this):timing based contact switch
        tci_container->force_task_map_["lf_force_task"]->UpdateDesiredToLocal(
            mpc_interface->des_lf_wrench_);

        // task
        // task_vector.push_back(task_map["lf_pos_task"]);
        // task_vector.push_back(task_map["lf_ori_task"]);
        lf_ori_quat_ = Eigen::Quaterniond(
            robot_->GetLinkIsometry(draco_link::l_foot_contact).linear());
      } else {
        // contact
        contact_vector.push_back(contact_map["rf_contact"]);
        sp_->b_rf_contact_ =
            true; // TODO(change this):timing based contact switch
        tci_container->force_task_map_["rf_force_task"]->UpdateDesiredToLocal(
            mpc_interface->des_rf_wrench_);
        // task
        // task_vector.push_back(task_map["rf_pos_task"]);
        // task_vector.push_back(task_map["rf_ori_task"]);
        rf_ori_quat_ = Eigen::Quaterniond(
            robot_->GetLinkIsometry(draco_link::r_foot_contact).linear());
      }
    } else {
      // in swing
      if (leg == 0) {
        task_vector.push_back(task_map["lf_pos_task"]);
        tci_container->task_map_["lf_pos_task"]->UpdateDesired(
            mpc_interface->des_foot_pos_[0], mpc_interface->des_foot_vel_[0],
            mpc_interface->des_foot_acc_[0]);
        // task_vector.push_back(task_map["lf_ori_task"]);
        // tci_container->task_map_["lf_ori_task"]->UpdateDesired(
        // lf_ori_quat_.coeffs(), Eigen::Vector3d::Zero(),
        // Eigen::Vector3d::Zero());
      } else {
        task_vector.push_back(task_map["rf_pos_task"]);
        tci_container->task_map_["rf_pos_task"]->UpdateDesired(
            mpc_interface->des_foot_pos_[1], mpc_interface->des_foot_vel_[1],
            mpc_interface->des_foot_acc_[1]);
        // task_vector.push_back(task_map["rf_ori_task"]);
        // tci_container->task_map_["rf_ori_task"]->UpdateDesired(
        // rf_ori_quat_.coeffs(), Eigen::Vector3d::Zero(),
        // Eigen::Vector3d::Zero());
      }
    }
  }
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

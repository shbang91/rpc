#include "controller/draco_controller/draco_state_machines/locomotion.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_task/draco_com_xy_task.hpp"
#include "controller/draco_controller/draco_task/draco_com_z_task.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/contact.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "controller/whole_body_controller/wbic/wbic.hpp"
#include "convex_mpc/convex_mpc_locomotion.hpp"

#if B_USE_ZMQ
#include "controller/draco_controller/draco_data_manager.hpp"
#endif

Locomotion::Locomotion(const StateId state_id, PinocchioRobotSystem *robot,
                       DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch) {
  util::PrettyConstructor(2, "Locomotion");

  sp_ = DracoStateProvider::GetStateProvider();

  // mpc gait
  gait_params_ = std::make_unique<GaitParams>();
  gait_command_ = std::make_shared<GaitCommand>();

  // contact states
  prev_contact_states_ << 1.0, 1.0; // both feet in contact
}

void Locomotion::FirstVisit() {
  std::cout << "draco_states: kLocomotion" << std::endl;
  state_machine_start_time_ = sp_->current_time_;

  // wbc reaction wrench smoother
  ctrl_arch_->tci_container_->qp_params_->W_force_rate_of_change_
      << W_force_rate_of_change_left_foot_,
      W_force_rate_of_change_right_foot_;

  // wbc reaction wrench tracking
  ctrl_arch_->tci_container_->qp_params_->W_delta_rf_ << W_delta_rf_lfoot_,
      W_delta_rf_rfoot_;

  // initialize convexMPC
  gait_command_->vel_xy_des[0] = gait_params_->x_vel_cmd_;
  gait_command_->vel_xy_des[1] = gait_params_->y_vel_cmd_;
  gait_command_->yaw_rate = gait_params_->yaw_rate_cmd_;
  // ctrl_arch_->convex_mpc_locomotion_->Initialize(*gait_command_,
  // sp_->des_body_height_);
  // ctrl_arch_->convex_mpc_locomotion_->Initialize(*gait_command_,
  // sp_->des_com_height_);
  double des_height = robot_->GetRobotComPos()[2];
  ctrl_arch_->convex_mpc_locomotion_->Initialize(*gait_command_, des_height);
  ctrl_arch_->convex_mpc_locomotion_->SetGait(gait_params_->gait_number_);
  ctrl_arch_->convex_mpc_locomotion_->SetSwingHeight(
      gait_params_->swing_height_);
  ctrl_arch_->convex_mpc_locomotion_->SetRaibertGain(
      gait_params_->raibert_gain_);
  ctrl_arch_->convex_mpc_locomotion_->SetHighSpeedTurningGain(
      gait_params_->high_speed_turning_gain_);
  ctrl_arch_->convex_mpc_locomotion_->SetFootLandingOffset(
      gait_params_->landing_foot_offset_);
  // TODO: check if this is true
  ctrl_arch_->convex_mpc_locomotion_->SetHipLocation(
      robot_->GetBaseToFootXYOffset());
}

void Locomotion::OneStep() {
  state_machine_time_ = sp_->current_time_ - state_machine_start_time_;

  const auto &mpc_interface = ctrl_arch_->convex_mpc_locomotion_;
  const auto &tci_container = ctrl_arch_->tci_container_;

  //=========================================================================
  // Gait command User Interrupt
  //=========================================================================
  if (b_increase_x_vel_) {
    gait_params_->x_vel_cmd_ += 0.1;
    mpc_interface->x_vel_cmd_ = gait_params_->x_vel_cmd_;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "Desired X vel: " << mpc_interface->x_vel_cmd_ << "m/s"
              << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    b_increase_x_vel_ = false;
  }
  if (b_increase_y_vel_) {
    gait_params_->y_vel_cmd_ += 0.1;
    mpc_interface->y_vel_cmd_ = gait_params_->y_vel_cmd_;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "Desired Y vel: " << mpc_interface->y_vel_cmd_ << "m/s"
              << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    b_increase_y_vel_ = false;
  }
  if (b_increase_yaw_vel_) {
    gait_params_->yaw_rate_cmd_ += 0.1;
    mpc_interface->yaw_rate_cmd_ = gait_params_->yaw_rate_cmd_;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "Desired Yaw vel: " << mpc_interface->yaw_rate_cmd_ << "rad/s"
              << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    b_increase_yaw_vel_ = false;
  }
  if (b_decrease_yaw_vel_) {
    gait_params_->yaw_rate_cmd_ -= 0.1;
    mpc_interface->yaw_rate_cmd_ = gait_params_->yaw_rate_cmd_;
    std::cout << "-----------------------------------" << std::endl;
    std::cout << "Desired Yaw vel: " << mpc_interface->yaw_rate_cmd_ << "rad/s"
              << std::endl;
    std::cout << "-----------------------------------" << std::endl;
    b_decrease_yaw_vel_ = false;
  }

  // set WBO & WBO ang vel
  mpc_interface->UpdateWBO(sp_->wbo_ypr_, sp_->wbo_ang_vel_);

  // solve convexMPC
  mpc_interface->Solve();

  // contact and task clear
  auto &contact_vector = ctrl_arch_->tci_container_->contact_vector_;
  auto &contact_map = ctrl_arch_->tci_container_->contact_map_;
  contact_vector.clear();
  auto &task_vector = ctrl_arch_->tci_container_->task_vector_;
  auto &task_map = ctrl_arch_->tci_container_->task_map_;
  task_vector.clear();

  // std::cout << "-------------------------------------------------" <<
  // std::endl; std::cout << "contact state: " <<
  // mpc_interface->contact_state_.transpose()
  //<< std::endl;

  // update foot task & contact for WBC
  for (int foot = 0; foot < foot_side::NumFoot; foot++) {
    if (mpc_interface->contact_state_[foot] > 0.0) // in contact
    {
      if (foot == foot_side::LFoot) {
        // contact(timing based contact switch)
        contact_vector.push_back(contact_map["lf_contact"]);
        sp_->b_lf_contact_ = true;
        tci_container->force_task_map_["lf_force_task"]->UpdateDesiredToLocal(
            mpc_interface->des_lf_wrench_);
        tci_container->contact_map_["lf_contact"]->SetMaxFz(
            ctrl_arch_->mpc_params_->fz_max_);

        // TEST
        if (prev_contact_states_[foot] == 0.0) {
          contact_map["lf_contact"]->SetDesiredPos(
              robot_->GetLinkIsometry(draco_link::l_foot_contact)
                  .translation());
          contact_map["lf_contact"]->SetDesiredOri(
              Eigen::Quaterniond(
                  robot_->GetLinkIsometry(draco_link::l_foot_contact).linear())
                  .normalized());
        }
        // TEST

        // task
        // task_vector.push_back(task_map["lf_pos_task"]);
        // task_vector.push_back(task_map["lf_ori_task"]);
        // lf_ori_quat_ = Eigen::Quaterniond(
        // robot_->GetLinkIsometry(draco_link::l_foot_contact).linear());
      } else if (foot == foot_side::RFoot) {
        // contact(timing based contact switch)
        contact_vector.push_back(contact_map["rf_contact"]);
        sp_->b_rf_contact_ = true;
        tci_container->force_task_map_["rf_force_task"]->UpdateDesiredToLocal(
            mpc_interface->des_rf_wrench_);
        tci_container->contact_map_["rf_contact"]->SetMaxFz(
            ctrl_arch_->mpc_params_->fz_max_);
        // task
        // task_vector.push_back(task_map["rf_pos_task"]);
        // task_vector.push_back(task_map["rf_ori_task"]);
        // rf_ori_quat_ = Eigen::Quaterniond(
        // robot_->GetLinkIsometry(draco_link::r_foot_contact).linear());
        //
        // TEST
        if (prev_contact_states_[foot] == 0.0) {
          contact_map["rf_contact"]->SetDesiredPos(
              robot_->GetLinkIsometry(draco_link::r_foot_contact)
                  .translation());
          contact_map["rf_contact"]->SetDesiredOri(
              Eigen::Quaterniond(
                  robot_->GetLinkIsometry(draco_link::r_foot_contact).linear())
                  .normalized());
        }
        // TEST
      }
    } else {
      // in swing
      if (foot == foot_side::LFoot) {
        // contact
        tci_container->contact_map_["lf_contact"]->SetMaxFz(0.01);
        tci_container->force_task_map_["lf_force_task"]->UpdateDesiredToLocal(
            mpc_interface->des_lf_wrench_);
        // task
        tci_container->task_map_["lf_pos_task"]->UpdateDesired(
            mpc_interface->des_foot_pos_[foot],
            mpc_interface->des_foot_vel_[foot],
            mpc_interface->des_foot_acc_[foot]);
        tci_container->task_map_["lf_ori_task"]->UpdateDesired(
            mpc_interface->des_foot_ori_[foot].coeffs(),
            mpc_interface->des_foot_ang_vel_[foot],
            mpc_interface->des_foot_ang_acc_[foot]);
      } else if (foot == foot_side::RFoot) {
        // contact
        tci_container->contact_map_["rf_contact"]->SetMaxFz(0.01);
        tci_container->force_task_map_["rf_force_task"]->UpdateDesiredToLocal(
            mpc_interface->des_rf_wrench_);
        // task
        tci_container->task_map_["rf_pos_task"]->UpdateDesired(
            mpc_interface->des_foot_pos_[foot],
            mpc_interface->des_foot_vel_[foot],
            mpc_interface->des_foot_acc_[foot]);
        tci_container->task_map_["rf_ori_task"]->UpdateDesired(
            mpc_interface->des_foot_ori_[foot].coeffs(),
            mpc_interface->des_foot_ang_vel_[foot],
            mpc_interface->des_foot_ang_acc_[foot]);
      }
    }
  }

  // update centroidal task
  Eigen::Vector3d zero_vec = Eigen::Vector3d::Zero();
  tci_container->task_map_["com_z_task"]->UpdateDesired(
      mpc_interface->des_body_pos_.tail<1>(),
      mpc_interface->des_body_vel_.tail<1>(), zero_vec.tail<1>());
  tci_container->task_map_["com_xy_task"]->UpdateDesired(
      mpc_interface->des_body_pos_.head<2>(),
      mpc_interface->des_body_vel_.head<2>(), Eigen::Vector2d::Zero());
  // util::WrapYawToPi(mpc_interface->des_body_rpy_);
  Eigen::Quaterniond des_body_quat = util::EulerZYXtoQuat(
      mpc_interface->des_body_rpy_[0], mpc_interface->des_body_rpy_[1],
      mpc_interface->des_body_rpy_[2]);
  Eigen::VectorXd des_body_quat_vec(4);
  des_body_quat_vec << des_body_quat.x(), des_body_quat.y(), des_body_quat.z(),
      des_body_quat.w();
  tci_container->task_map_["torso_ori_task"]->UpdateDesired(
      des_body_quat_vec, mpc_interface->des_body_ang_vel_,
      Eigen::Vector3d::Zero());

  // WBC task hierarchy
  if (mpc_interface->swing_state_[foot_side::LFoot] > 0) {
    task_vector.push_back(task_map["lf_pos_task"]);
    task_vector.push_back(task_map["lf_ori_task"]);
  }
  if (mpc_interface->swing_state_[foot_side::RFoot] > 0) {
    task_vector.push_back(task_map["rf_pos_task"]);
    task_vector.push_back(task_map["rf_ori_task"]);
  }
  task_vector.push_back(task_map["com_z_task"]);
  task_vector.push_back(task_map["com_xy_task"]);
  task_vector.push_back(task_map["torso_ori_task"]);
  // task_vector.push_back(task_map["wbo_task"]);
  task_vector.push_back(task_map["upper_body_task"]);

  // update contact states
  prev_contact_states_ = mpc_interface->contact_state_;

#if B_USE_ZMQ
  // for meshcat visualization (only visualize desired base & foot trajectories
  // for computing variable inertia calculation)
  // TODO: visualize chaning target footstep location
  if (sp_->count_ % sp_->data_save_freq_ == 0) {
    DracoDataManager *dm = DracoDataManager::GetDataManager();

    dm->data_->des_com_traj.clear();
    dm->data_->des_torso_ori_traj.clear();
    dm->data_->des_com_traj.reserve(mpc_interface->des_state_traj_.size());
    dm->data_->des_torso_ori_traj.reserve(
        mpc_interface->des_state_traj_.size());
    for (const auto &des_state : mpc_interface->des_state_traj_) {
      dm->data_->des_com_traj.push_back(des_state.segment<3>(3));
      dm->data_->des_torso_ori_traj.push_back(des_state.segment<3>(0));
    }

    dm->data_->des_lf_pos_traj.clear();
    dm->data_->des_lf_pos_traj.reserve(
        mpc_interface->des_foot_pos_traj_[0].size());
    dm->data_->des_rf_pos_traj.clear();
    dm->data_->des_rf_pos_traj.reserve(
        mpc_interface->des_foot_pos_traj_[1].size());
    dm->data_->des_lf_ori_traj.clear();
    dm->data_->des_lf_ori_traj.reserve(
        mpc_interface->des_foot_ori_traj_[0].size());
    dm->data_->des_rf_ori_traj.clear();
    dm->data_->des_rf_ori_traj.reserve(
        mpc_interface->des_foot_ori_traj_[1].size());
    for (const auto &lf_pos : mpc_interface->des_foot_pos_traj_[0])
      dm->data_->des_lf_pos_traj.push_back(lf_pos);
    for (const auto &rf_pos : mpc_interface->des_foot_pos_traj_[1])
      dm->data_->des_rf_pos_traj.push_back(rf_pos);
    for (const auto &lf_ori : mpc_interface->des_foot_ori_traj_[0])
      dm->data_->des_lf_ori_traj.push_back(lf_ori);
    for (const auto &rf_ori : mpc_interface->des_foot_ori_traj_[1])
      dm->data_->des_rf_ori_traj.push_back(rf_ori);
  }
#endif
}

bool Locomotion::EndOfState() { return false; }

void Locomotion::LastVisit() {}

StateId Locomotion::GetNextState() {}

void Locomotion::SetParameters(const YAML::Node &node) {
  try {
    // wbc params setting
    util::ReadParameter(node["wbc"]["qp"], "W_force_rate_of_change_left_foot",
                        W_force_rate_of_change_left_foot_);
    util::ReadParameter(node["wbc"]["qp"], "W_force_rate_of_change_right_foot",
                        W_force_rate_of_change_right_foot_);
    util::ReadParameter(node["wbc"]["qp"],
                        "W_delta_rf_left_foot_in_contact_mpc",
                        W_delta_rf_lfoot_);
    util::ReadParameter(node["wbc"]["qp"],
                        "W_delta_rf_right_foot_in_contact_mpc",
                        W_delta_rf_rfoot_);
    YAML::Node mpc_cfg =
        YAML::LoadFile(THIS_COM "config/draco/MPC_LOCOMOTION.yaml");
    gait_params_->x_vel_cmd_ =
        util::ReadParameter<double>(mpc_cfg["gait"], "x_vel_cmd");
    gait_params_->y_vel_cmd_ =
        util::ReadParameter<double>(mpc_cfg["gait"], "y_vel_cmd");
    gait_params_->yaw_rate_cmd_ =
        util::ReadParameter<double>(mpc_cfg["gait"], "yaw_rate_cmd");
    gait_params_->gait_number_ =
        util::ReadParameter<int>(mpc_cfg["gait"], "gait_number");
    gait_params_->swing_height_ =
        util::ReadParameter<double>(mpc_cfg["swing_foot"], "height");
    gait_params_->raibert_gain_ =
        util::ReadParameter<double>(mpc_cfg["swing_foot"], "raibert_gain");
    gait_params_->high_speed_turning_gain_ = util::ReadParameter<double>(
        mpc_cfg["swing_foot"], "high_speed_turning_gain");
    gait_params_->landing_foot_offset_ = util::ReadParameter<Eigen::Vector3d>(
        mpc_cfg["swing_foot"], "landing_foot_offset");
  } catch (std::runtime_error &e) {
    std::cerr << "Error reading parameter [ " << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

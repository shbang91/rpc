#include "controller/draco_controller/draco_state_machines/teleop_manipulation.hpp"
#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_rs_teleop_handler.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_tci_container.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/managers/hand_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/max_normal_force_trajectory_manager.hpp"
#include "controller/whole_body_controller/managers/task_hierarchy_manager.hpp"
#include "controller/whole_body_controller/task.hpp"

TeleopManipulation::TeleopManipulation(StateId state_id,
                                       PinocchioRobotSystem *robot,
                                       DracoControlArchitecture *ctrl_arch)
    : StateMachine(state_id, robot), ctrl_arch_(ctrl_arch),
      b_first_visit_(true), b_transition_(false), b_teleop_mode_(false),
      b_initialized_(false) {
  util::PrettyConstructor(2, "TeleopManipulation");
  sp_ = DracoStateProvider::GetStateProvider();

  rs_commands_ = new DracoRSCommands();

  target_rh_iso_.setIdentity();
  target_lh_iso_.setIdentity();
}

TeleopManipulation::~TeleopManipulation() { delete rs_commands_; }

void TeleopManipulation::FirstVisit() {
  // Do nothing for now
}

void TeleopManipulation::OneStep() {
  // Do nothing when Initialize and standup phase
  if (sp_->state_ != draco_states::kInitialize &&
      sp_->state_ != draco_states::kDoubleSupportStandUp) {

    //=====================================================
    // first visit loop
    //=====================================================
    // Get new Command at every teleop freqeuncy loop
    if (sp_->count_ % teleop_freq_ == 0) {
      // Get commands from zmq and send the interrupt
      teleop_handler_->ReceiveCommands(rs_commands_);
      if (teleop_handler_->IsReady() && b_first_visit_) {
        std::cout << "draco_states::kTeleopManipulation" << std::endl;
        state_machine_start_time_ = sp_->current_time_;
        // set the desired hand pos as the current pos and increase the task
        // hierarchy
        target_rh_iso_.translation() =
            robot_->GetLinkIsometry(draco_link::r_hand_contact).translation();
        target_rh_iso_.linear() =
            robot_->GetLinkIsometry(draco_link::r_hand_contact).linear();
        ctrl_arch_->rh_SE3_tm_->UpdateDesired(target_rh_iso_);

        initial_torso_to_rh_iso_.translation() =
            robot_->GetLinkIsometry(draco_link::torso_com_link)
                .linear()
                .transpose() *
            (robot_->GetLinkIsometry(draco_link::r_hand_contact).translation() -
             robot_->GetLinkIsometry(draco_link::torso_com_link).translation());
        initial_torso_to_rh_iso_.linear() =
            robot_->GetLinkIsometry(draco_link::torso_com_link)
                .linear()
                .transpose() *
            robot_->GetLinkIsometry(draco_link::r_hand_contact).linear();

        initial_torso_to_lh_iso_.translation() =
            robot_->GetLinkIsometry(draco_link::torso_com_link)
                .linear()
                .transpose() *
            (robot_->GetLinkIsometry(draco_link::l_hand_contact).translation() -
             robot_->GetLinkIsometry(draco_link::torso_com_link).translation());
        initial_torso_to_lh_iso_.linear() =
            robot_->GetLinkIsometry(draco_link::torso_com_link)
                .linear()
                .transpose() *
            robot_->GetLinkIsometry(draco_link::l_hand_contact).linear();

        ctrl_arch_->rh_pos_hm_->InitializeRampToMax(transition_duration_);
        ctrl_arch_->rh_ori_hm_->InitializeRampToMax(transition_duration_);
        b_transition_ = true;

      } else if (teleop_handler_->IsReady() && b_teleop_mode_) {
        // update the desired hand pos as the teleop commands
        // trajectory setting (time, desired pos)

        Eigen::Isometry3d target_iso = Eigen::Isometry3d::Identity();

        Eigen::Isometry3d torso_com_iso =
            robot_->GetLinkIsometry(draco_link::torso_com_link);
        target_iso.translation() =
            torso_com_iso.linear() *
                (initial_torso_to_rh_iso_.translation() + rs_commands_->pos_) +
            torso_com_iso.translation();
        target_iso.linear() = torso_com_iso.linear() *
                              initial_torso_to_rh_iso_.linear() *
                              rs_commands_->quat_;

        ctrl_arch_->rh_SE3_tm_->InitializeHandTrajectory(
            target_iso, sp_->current_time_, moving_duration_, b_initialized_);
        b_initialized_ = true;
      }
    }

    //=====================================================
    // one step update loop
    //=====================================================
    // execute the command every control loop
    if (teleop_handler_->IsReady() && b_transition_) {
      b_first_visit_ = false;
      state_machine_time_ = sp_->current_time_ - state_machine_start_time_;
      ctrl_arch_->rh_pos_hm_->UpdateRampToMax(state_machine_time_);
      ctrl_arch_->rh_ori_hm_->UpdateRampToMax(state_machine_time_);
      if (state_machine_time_ > transition_duration_) {
        b_transition_ = false;
        b_teleop_mode_ = true;
      }
    } else if (teleop_handler_->IsReady() && b_teleop_mode_) {
      // execute the desired trajectory
      ctrl_arch_->rh_SE3_tm_->UpdateHandPose(sp_->current_time_);

      // gripper command
      sp_->b_recv_gripper_cmd_ = false;
      if (rs_commands_->b_grasp_ != b_prev_grasp_) {
        sp_->b_recv_gripper_cmd_ = true;

        // update gripper target pos
        if (rs_commands_->b_grasp_)
          sp_->gripper_pos_cmd_["right"] = right_gripper_target_pos_["grasp"];
        else
          sp_->gripper_pos_cmd_["right"] = right_gripper_target_pos_["release"];

        b_prev_grasp_ = rs_commands_->b_grasp_;
      }
    }
  }
}

bool TeleopManipulation::EndOfState() { return false; }

void TeleopManipulation::LastVisit() {}

StateId TeleopManipulation::GetNextState() {
  return draco_states::kTeleopManipulation;
}

void TeleopManipulation::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "teleop_freq", teleop_freq_);
    // construct teleop handler
    teleop_handler_ = std::make_unique<DracoRSTeleopHandler>();
    std::cout << "Connecting to Teleop Socket...." << std::endl;
    const std::string ip_address =
        util::ReadParameter<std::string>(node, "teleop_ip_address");
    if (teleop_handler_->InitializeSocket(ip_address))
      std::cout << "Connected to Teleop Socket!" << std::endl;

    util::ReadParameter(node["state_machine"]["teleop_manipulation"],
                        "transition_duration", transition_duration_);
    util::ReadParameter(node["state_machine"]["teleop_manipulation"],
                        "moving_duration", moving_duration_);
    double tmp;
    util::ReadParameter(
        node["state_machine"]["teleop_manipulation"]["gripper_control"]["left"],
        "grasp", tmp);
    left_gripper_target_pos_["grasp"] = tmp;
    util::ReadParameter(
        node["state_machine"]["teleop_manipulation"]["gripper_control"]["left"],
        "release", tmp);
    left_gripper_target_pos_["release"] = tmp;
    util::ReadParameter(node["state_machine"]["teleop_manipulation"]
                            ["gripper_control"]["right"],
                        "grasp", tmp);
    right_gripper_target_pos_["grasp"] = tmp;
    util::ReadParameter(node["state_machine"]["teleop_manipulation"]
                            ["gripper_control"]["right"],
                        "release", tmp);
    right_gripper_target_pos_["release"] = tmp;

  } catch (const std::runtime_error &ex) {
    std::cerr << "Error Reading Parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }
}

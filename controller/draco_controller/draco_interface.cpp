#include "configuration.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/draco_controller/draco_kf_state_estimator.hpp"
#include "controller/draco_controller/draco_state_estimator.hpp"

#include "controller/draco_controller/draco_control_architecture.hpp"
#include "controller/draco_controller/draco_interface.hpp"
#include "controller/draco_controller/draco_interrupt_handler.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/draco_controller/draco_task_gain_handler.hpp"

#include "controller/draco_controller/draco_definition.hpp"
#include "util/util.hpp"
#include <chrono>

#if B_USE_ZMQ
#include "controller/draco_controller/draco_data_manager.hpp"
#endif

#if B_USE_VR_TELEOP
#include "controller/draco_controller/draco_vr_teleop_manager.hpp"
#endif

DracoInterface::DracoInterface() : Interface() {
  std::string border = "=";
  for (unsigned int i = 0; i < 79; ++i)
    border += "=";
  util::ColorPrint(color::kBoldRed, border);
  util::PrettyConstructor(0, "DracoInterface");

  sp_ = DracoStateProvider::GetStateProvider();
  try {
    YAML::Node cfg =
        YAML::LoadFile(THIS_COM "config/draco/pnc.yaml"); // get yaml node

    sp_->servo_dt_ =
        util::ReadParameter<double>(cfg, "servo_dt"); // set control frequency

    sp_->data_save_freq_ = util::ReadParameter<int>(cfg, "data_save_freq");
    sp_->vr_teleop_freq_ = util::ReadParameter<int>(cfg, "vr_teleop_freq");
    sp_->b_use_kf_state_estimator_ =
        util::ReadParameter<bool>(cfg["state_estimator"], "kf");

#if B_USE_ZMQ
    if (!DracoDataManager::GetDataManager()->IsInitialized()) {
      std::string socket_address =
          util::ReadParameter<std::string>(cfg, "ip_address");
      DracoDataManager::GetDataManager()->InitializeSocket(
          socket_address); // initalize data publisher
    }
#endif
#if B_USE_VR_TELEOP
    std::cout << "Connecting to VR socket..." << std::endl;
    std::string socket_address =
        util::ReadParameter<std::string>(cfg, "vr_ip_address");
    std::cout << "read address" << std::endl;
    DracoVRTeleopManager::GetVRTeleopManager()->InitializeTeleopSocket(
        socket_address);
    std::cout << "Connected to VR socket" << std::endl;
#endif

  } catch (const std::runtime_error &ex) {
    std::cerr << "Error Reading Parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }

  // robot_ =
  // new PinocchioRobotSystem(THIS_COM "robot_model/draco/draco_modified.urdf",
  // THIS_COM "robot_model/draco", false, false);
  robot_ = new PinocchioRobotSystem(THIS_COM
                                    "robot_model/draco/draco3_big_feet.urdf",
                                    THIS_COM "robot_model/draco", false, false);
  se_ = new DracoStateEstimator(robot_);
  se_kf_ = new DracoKFStateEstimator(robot_);
  ctrl_arch_ = new DracoControlArchitecture(robot_);
  interrupt_handler_ = new DracoInterruptHandler(
      static_cast<DracoControlArchitecture *>(ctrl_arch_));
  task_gain_handler_ = new DracoTaskGainHandler(
      static_cast<DracoControlArchitecture *>(ctrl_arch_));

  // assume start with double support
  sp_->b_lf_contact_ = true;
  sp_->b_rf_contact_ = true;
}

DracoInterface::~DracoInterface() {
  delete robot_;
  delete se_;
  delete se_kf_;
  delete ctrl_arch_;
  delete interrupt_handler_;
}

void DracoInterface::GetCommand(void *sensor_data, void *command_data) {
  sp_->count_ = count_;
  sp_->current_time_ = static_cast<double>(count_) * sp_->servo_dt_;
  sp_->state_ = ctrl_arch_->state();
  sp_->prev_state_ = ctrl_arch_->prev_state();

  DracoSensorData *draco_sensor_data =
      static_cast<DracoSensorData *>(sensor_data);
  DracoCommand *draco_command = static_cast<DracoCommand *>(command_data);

#if B_USE_VR_TELEOP
  if (sp_->count_ % sp_->vr_teleop_freq_ == 0) {
    // Get commands from zmq, send interrupt
    DracoVRCommands cmd =
        DracoVRTeleopManager::GetVRTeleopManager()->ReceiveCommands();
    Eigen::Vector3d test_rh_pos;
    Eigen::Vector3d test_lh_pos;
    Eigen::Quaterniond test_rh_quat;
    Eigen::Quaterniond test_lh_quat;
    if (DracoVRTeleopManager::GetVRTeleopManager()->isReady()) {
      test_rh_pos = cmd.rh_pos;
      test_lh_pos = cmd.lh_pos;
      test_rh_quat = cmd.rh_ori;
      test_lh_quat = cmd.lh_ori;
    } else {
      test_rh_pos << 0.35, -0.25, 0.0;
      test_lh_pos << 0.35, 0.25, 0.0;
      test_rh_quat = Eigen::AngleAxisd(
          0.0,
          Eigen::Vector3d::UnitZ()); // TEST VALUES WITH UnitX, UnitY, UnitZ
      test_lh_quat = Eigen::AngleAxisd(
          0.0,
          Eigen::Vector3d::UnitZ()); // TEST VALUES WITH UnitX, UnitY, UnitZ
    }
    //_ProcessVRInput(&cmd, sensor_data);
    test_rh_pos[2] += .5;
    test_lh_pos[2] += .5;
    // std::cout << "left\n " << test_lh_pos << std::endl;
    // std::cout << "right\n " << test_rh_pos << std::endl;

    Eigen::VectorXd clamped_rh_pos(3);
    Eigen::VectorXd clamped_lh_pos(3);

    Eigen::Vector3d target_rh_pos;
    Eigen::Vector3d target_lh_pos;
    Eigen::Quaterniond target_rh_quat;
    Eigen::Quaterniond target_lh_quat;

    Eigen::Quaterniond zero_rh_quat_(0.707, 0.0, -0.707,
                                     0.0); // THIS IS IN THE ORDER OF W, X, Y, Z
    Eigen::Quaterniond zero_lh_quat_(0.707, 0.0, -0.707,
                                     0.0); // THIS IS IN THE ORDER OF W, X, Y, Z

    clamped_lh_pos[0] = std::min(std::max(test_lh_pos[0], 0.25), 0.55);
    clamped_lh_pos[1] = std::min(std::max(test_lh_pos[1], -0.15), 0.45);
    clamped_lh_pos[2] = std::min(std::max(test_lh_pos[2], -0.1), 0.5);

    clamped_rh_pos[0] = std::min(std::max(test_rh_pos[0], 0.25), 0.55);
    clamped_rh_pos[1] = std::min(std::max(test_rh_pos[1], -0.45), 0.15);
    clamped_rh_pos[2] = std::min(std::max(test_rh_pos[2], -0.1), 0.5);

    // std::cout << "clamped left\n " << clamped_lh_pos << std::endl;
    // std::cout << "clamped right\n " << clamped_rh_pos << std::endl;

    Eigen::Isometry3d torso_iso =
        robot_->GetLinkIsometry(draco_link::torso_com_link);
    Eigen::Vector3d base_pos = torso_iso.translation();
    Eigen::Matrix3d rot_world_to_base = torso_iso.linear();
    Eigen::Quaterniond base_quat(rot_world_to_base);

    target_rh_pos = rot_world_to_base * clamped_rh_pos + base_pos;
    target_lh_pos = rot_world_to_base * clamped_lh_pos + base_pos;
    target_rh_quat = base_quat * test_rh_quat * zero_rh_quat_;
    target_lh_quat = base_quat * test_lh_quat * zero_lh_quat_;

    ctrl_arch_->background_manipulation_->target_rh_pos_ << target_rh_pos;
    ctrl_arch_->background_manipulation_->target_lh_pos_ << target_lh_pos;

    ctrl_arch_->background_manipulation_->target_rh_ori_[0] =
        target_rh_quat.x();
    ctrl_arch_->background_manipulation_->target_rh_ori_[1] =
        target_rh_quat.y();
    ctrl_arch_->background_manipulation_->target_rh_ori_[2] =
        target_rh_quat.z();
    ctrl_arch_->background_manipulation_->target_rh_ori_[3] =
        target_rh_quat.w();

    ctrl_arch_->background_manipulation_->target_lh_ori_[0] =
        target_lh_quat.x();
    ctrl_arch_->background_manipulation_->target_lh_ori_[1] =
        target_lh_quat.y();
    ctrl_arch_->background_manipulation_->target_lh_ori_[2] =
        target_lh_quat.z();
    ctrl_arch_->background_manipulation_->target_lh_ori_[3] =
        target_lh_quat.w();
  }
#endif
  // if (count_ <= waiting_count_) {
  // for simulation without state estimator
  // se_->UpdateGroundTruthSensorData(draco_sensor_data);
  // se_->Initialize(draco_sensor_data);
  // this->_SafeCommand(draco_sensor_data, draco_command);
  //} else {

  // for simulation without state estimator
  // se_->UpdateGroundTruthSensorData(draco_sensor_data);

  if (sp_->b_use_kf_state_estimator_) {
    sp_->state_ == draco_states::kInitialize
        ? se_kf_->Initialize(draco_sensor_data)
        : se_kf_->Update(draco_sensor_data);
  } else {
    sp_->state_ == draco_states::kInitialize
        ? se_->Initialize(draco_sensor_data)
        : se_->Update(draco_sensor_data);
  }

  // process interrupt & task gains
  if (interrupt_handler_->IsSignalReceived())
    interrupt_handler_->Process();
  if (task_gain_handler_->IsSignalReceived())
    task_gain_handler_->Process();

  // get control command
  ctrl_arch_->GetCommand(draco_command);

#if B_USE_ZMQ
  if (sp_->count_ % sp_->data_save_freq_ == 0) {
    DracoDataManager *dm = DracoDataManager::GetDataManager();
    dm->data_->time_ = sp_->current_time_;
    dm->data_->phase_ = sp_->state_;
    dm->SendData();
  }
#endif

  count_++;
}

void DracoInterface::_SafeCommand(DracoSensorData *data,
                                  DracoCommand *command) {
  command->joint_pos_cmd_ = data->joint_pos_;
  command->joint_vel_cmd_.setZero();
  command->joint_trq_cmd_.setZero();
}

// void DracoInterface::_ProcessVRInput(DracoVRCommands* cmd, void *sensor_data)
// {
//
//
//   ctrl_arch_->background_manipulation_->target_lh_pos_<< cmd->lh_pos;
//   ctrl_arch_->background_manipulation_->target_rh_pos_<< cmd->rh_pos;
//
//   std::cout << target_lh_pos << std::endl;
//
//   /*
//   ctrl_arch_->background_manipulation_->target_rh_ori_<< target_rh_quat.x(),
//       target_rh_quat.y(),
//       target_rh_quat.z(),
//       target_rh_quat.w();
//   ctrl_arch_->background_manipulation_->target_lh_ori_<< target_lh_quat.x(),
//       target_lh_quat.y(),
//       target_lh_quat.z(),
//       target_lh_quat.w();
//   */
//
//
//   /*
//   if (cmd->l_button) {
//       interrupt_handler_->PressEight();
//   } else if (cmd->l_pad) {
//       interrupt_handler_->PressTwo();
//   } else if (cmd->r_button) {
//       interrupt_handler_->PressFour();
//   } else if (cmd->r_pad) {
//       interrupt_handler_->PressSix();
//   } else if (cmd->l_trigger) {
//       interrupt_handler_->PressSeven();
//   } else if (cmd->r_trigger) {
//       interrupt_handler_->PressNine();
//   }
//   */
// }

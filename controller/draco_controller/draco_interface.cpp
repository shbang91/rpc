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
      DracoVRTeleopManager::GetVRTeleopManager()->InitializeTeleopSocket(socket_address);
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
#if B_USE_VR_TELEOP
  if (sp_->count_ % sp_->vr_teleop_freq_ == 0) {
      // Get commands from zmq, send interrupt
      DracoVRCommands cmd = DracoVRTeleopManager::GetVRTeleopManager()->ReceiveCommands();
      _ProcessVRInput(&cmd);
  }
#endif
  sp_->count_ = count_;
  sp_->current_time_ = static_cast<double>(count_) * sp_->servo_dt_;
  sp_->state_ = ctrl_arch_->state();
  sp_->prev_state_ = ctrl_arch_->prev_state();

  DracoSensorData *draco_sensor_data =
      static_cast<DracoSensorData *>(sensor_data);
  DracoCommand *draco_command = static_cast<DracoCommand *>(command_data);

  // NEED TO UPDATE THESE VALUES
  // OUTPUTS
  // ctrl_arch_->target_rh_pos_
  // ctrl_arch_->target_lh_pos_
  // ctrl_arch_->target_rh_ori_
  // ctrl_arch_->target_lh_ori_

  // REFER
  // draco_sensor_data->base_joint_pos_;
  // draco_sensor_data->base_joint_quat_; // x, y, z, w order

  // INPUTS
  // vr_command_->target_rh_pos_;
  // vr_command_->global_lh_pos_;
  // vr_command_->global_rh_ori_;
  // vr_command_->global_lh_ori_;

  Eigen::Vector3d test_rh_pos(0.3, -0.3, 0.3);
  Eigen::Vector3d test_lh_pos(0.3, 0.3, 0.3);
  Eigen::Quaterniond test_rh_quat;
  Eigen::Quaterniond test_lh_quat;
  test_rh_quat = Eigen::AngleAxisd(0.125*3.14, Eigen::Vector3d::UnitZ()); // TEST VALUES WITH UnitX, UnitY, UnitZ
  test_lh_quat = Eigen::AngleAxisd(0.125*3.14, Eigen::Vector3d::UnitZ()); // TEST VALUES WITH UnitX, UnitY, UnitZ

  Eigen::Vector3d target_rh_pos;
  Eigen::Vector3d target_lh_pos;
  Eigen::Quaterniond target_rh_quat;
  Eigen::Quaterniond target_lh_quat;

  Eigen::Vector3d base_pos;
  Eigen::Quaterniond base_quat;

  Eigen::Quaterniond zero_rh_quat_(0.707, 0.0, -0.707, 0.0); //THIS IS IN THE ORDER OF W, X, Y, Z
  Eigen::Quaterniond zero_lh_quat_(0.707, 0.0, -0.707, 0.0); //THIS IS IN THE ORDER OF W, X, Y, Z

  Eigen::Matrix3d rot_word_to_base;

  base_pos = draco_sensor_data->base_joint_pos_;
  base_quat.x() = draco_sensor_data->base_joint_quat_[0];
  base_quat.y() = draco_sensor_data->base_joint_quat_[1];
  base_quat.z() = draco_sensor_data->base_joint_quat_[2];
  base_quat.w() = draco_sensor_data->base_joint_quat_[3];

  rot_word_to_base = base_quat.toRotationMatrix();
  target_rh_pos = rot_word_to_base * test_rh_pos + base_pos;
  target_lh_pos = rot_word_to_base * test_lh_pos + base_pos;
  target_rh_quat = base_quat * test_rh_quat * zero_rh_quat_;
  target_lh_quat = base_quat * test_lh_quat * zero_lh_quat_;

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

void DracoInterface::_ProcessVRInput(DracoVRCommands* cmd) {

  ctrl_arch_->background_manipulation_->target_lh_pos_<< cmd->lh_pos;
  // YOU MAY NEED TO REMOVE THE OFFSET FOR X AND Y//
  ctrl_arch_->background_manipulation_->target_lh_pos_(0) += 0.1;
  ctrl_arch_->background_manipulation_->target_lh_pos_(1) += 0.1;
  // YOU MAY NEED TO REMOVE THE OFFSET FOR X AND Y//
  ctrl_arch_->background_manipulation_->target_lh_pos_(2) += 1.0;

  ctrl_arch_->background_manipulation_->target_rh_pos_<< cmd->rh_pos;
  // YOU MAY NEED TO REMOVE THE OFFSET FOR X AND Y//
  ctrl_arch_->background_manipulation_->target_rh_pos_(0) += 0.3;
  ctrl_arch_->background_manipulation_->target_rh_pos_(1) += -0.1;
  // YOU MAY NEED TO REMOVE THE OFFSET FOR X AND Y//
  ctrl_arch_->background_manipulation_->target_rh_pos_(2) += 1.0;
    //std::cout << ctrl_arch_->background_manipulation_->target_lh_pos_ << std::endl;
    //std::cout << ctrl_arch_->background_manipulation_->target_rh_pos_ << std::endl;
  
  /*
  ctrl_arch_->background_manipulation_->target_rh_ori_<< target_rh_quat.x(), 
      target_rh_quat.y(),
      target_rh_quat.z(), 
      target_rh_quat.w();
  ctrl_arch_->background_manipulation_->target_lh_ori_<< target_lh_quat.x(), 
      target_lh_quat.y(),
      target_lh_quat.z(), 
      target_lh_quat.w();
  */


  /*
  if (cmd->l_button) {
      interrupt_handler_->PressEight();
  } else if (cmd->l_pad) {
      interrupt_handler_->PressTwo();
  } else if (cmd->r_button) {
      interrupt_handler_->PressFour();
  } else if (cmd->r_pad) {
      interrupt_handler_->PressSix();
  } else if (cmd->l_trigger) {
      interrupt_handler_->PressSeven();
  } else if (cmd->r_trigger) {
      interrupt_handler_->PressNine();
  }
  */
}

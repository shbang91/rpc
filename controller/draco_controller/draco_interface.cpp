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

#if B_USE_ZMQ
#include "controller/draco_controller/draco_data_manager.hpp"
#endif

DracoInterface::DracoInterface() : Interface() {
  std::string border = "=";
  for (unsigned int i = 0; i < 79; ++i)
    border += "=";
  util::ColorPrint(color::kBoldRed, border);
  util::PrettyConstructor(0, "DracoInterface");

  sp_ = DracoStateProvider::GetStateProvider();
  Eigen::Vector3d com_offset = Eigen::Vector3d::Zero();
  try {
    YAML::Node cfg =
        YAML::LoadFile(THIS_COM "config/draco/pnc.yaml"); // get yaml node

    sp_->servo_dt_ =
        util::ReadParameter<double>(cfg, "servo_dt"); // set control frequency

    sp_->data_save_freq_ = util::ReadParameter<int>(cfg, "data_save_freq");
    sp_->b_use_kf_state_estimator_ =
        util::ReadParameter<bool>(cfg["state_estimator"], "kf");
    com_offset = util::ReadParameter<Eigen::Vector3d>(
            cfg["controller"], "com_offset");

#if B_USE_ZMQ
    if (!DracoDataManager::GetDataManager()->IsInitialized()) {
      std::string socket_address =
          util::ReadParameter<std::string>(cfg, "ip_address");
      DracoDataManager::GetDataManager()->InitializeSocket(
          socket_address); // initalize data publisher
    }
#endif

  } catch (const std::runtime_error &ex) {
    std::cerr << "Error Reading Parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }

  // robot_ =
  // new PinocchioRobotSystem(THIS_COM "robot_model/draco/draco_modified.urdf",
  // THIS_COM "robot_model/draco", false, false);
  robot_ =
      new PinocchioRobotSystem(THIS_COM "robot_model/draco/draco_modified.urdf",
                               THIS_COM "robot_model/draco", false, false);
  robot_->SetRobotComOffset(com_offset);
  se_ = new DracoStateEstimator(robot_);
//  se_kf_ = new DracoKFStateEstimator(robot_);
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
//  delete se_kf_;
  delete ctrl_arch_;
  delete interrupt_handler_;
  delete task_gain_handler_;
}

void DracoInterface::GetCommand(void *sensor_data, void *command_data) {
  sp_->count_ = count_;
  sp_->current_time_ = static_cast<double>(count_) * sp_->servo_dt_;
  sp_->state_ = ctrl_arch_->state();
  sp_->prev_state_ = ctrl_arch_->prev_state();

  DracoSensorData *draco_sensor_data =
      static_cast<DracoSensorData *>(sensor_data);
  DracoCommand *draco_command = static_cast<DracoCommand *>(command_data);

  // if (count_ <= waiting_count_) {
  // for simulation without state estimator
  // se_->UpdateGroundTruthSensorData(draco_sensor_data);
  // se_->Initialize(draco_sensor_data);
  // this->_SafeCommand(draco_sensor_data, draco_command);
  //} else {

  // for simulation without state estimator
  // se_->UpdateGroundTruthSensorData(draco_sensor_data);

  if (sp_->b_use_kf_state_estimator_) {
//    sp_->state_ == draco_states::kInitialize
 //       ? se_kf_->Initialize(draco_sensor_data)
 //       : se_kf_->Update(draco_sensor_data);
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

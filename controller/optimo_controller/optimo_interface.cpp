#include "configuration.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "controller/optimo_controller/optimo_interface.hpp"
#include "controller/optimo_controller/optimo_state_estimator.hpp"

#include "controller/optimo_controller/optimo_control_architecture.hpp"
#include "controller/optimo_controller/optimo_interrupt_handler.hpp"
#include "controller/optimo_controller/optimo_state_provider.hpp"
#include "controller/optimo_controller/optimo_task_gain_handler.hpp"

#include "controller/optimo_controller/optimo_definition.hpp"

// TODO:
#if B_USE_ZMQ
// #include "controller/optimo_controller/optimo_data_manager.hpp"
#endif

OptimoInterface::OptimoInterface() : Interface() {
  std::string border = "=";
  for (unsigned int i = 0; i < 79; ++i)
    border += "=";
  util::ColorPrint(color::kBoldRed, border);
  util::PrettyConstructor(0, "OptimoInterface");

  sp_ = OptimoStateProvider::GetStateProvider();

  // initialize robot model
  robot_ = new PinocchioRobotSystem(THIS_COM "robot_model/optimo/optimo.urdf",
                                    THIS_COM "robot_model/optimo", true, false);

  // set control parameters
  this->_SetParameters();

  se_ = new OptimoStateEstimator(robot_);

  ctrl_arch_ = new OptimoControlArchitecture(robot_, cfg_);

  interrupt_handler_ = new OptimoInterruptHandler(
      static_cast<OptimoControlArchitecture *>(ctrl_arch_));
  task_gain_handler_ = new OptimoTaskGainHandler(
      static_cast<OptimoControlArchitecture *>(ctrl_arch_));
}

OptimoInterface::~OptimoInterface() {
  delete robot_;
  delete se_;
  delete ctrl_arch_;
  delete interrupt_handler_;
  delete task_gain_handler_;
}

void OptimoInterface::GetCommand(void *sensor_data, void *command_data) {
  sp_->count_ = count_;
  sp_->current_time_ = static_cast<double>(count_) * sp_->servo_dt_;
  sp_->state_ = ctrl_arch_->manipstate();
  sp_->prev_state_ = ctrl_arch_->prev_manipstate();

  OptimoSensorData *optimo_sensor_data =
      static_cast<OptimoSensorData *>(sensor_data);

  OptimoCommand *optimo_command = static_cast<OptimoCommand *>(command_data);

  // update sensor data to pinocchio model using state estimator class
  se_->Update(optimo_sensor_data);

  // process interrupt & task gains
  if (interrupt_handler_->IsSignalReceived()) {
    interrupt_handler_->Process();
  }
  if (task_gain_handler_->IsSignalReceived())
    task_gain_handler_->Process();

  // get control command
  ctrl_arch_->GetCommand(optimo_command);

  count_++;
}
void OptimoInterface::_SetParameters() {
  try {
    // select test environment
    YAML::Node interface_cfg =
        YAML::LoadFile(THIS_COM "config/optimo/INTERFACE.yaml");
    std::string test_env_name =
        util::ReadParameter<std::string>(interface_cfg, "test_env_name");
    std::cout << "============================================" << '\n';
    std::cout << "TEST ENV: " << test_env_name << '\n';
    std::cout << "============================================" << '\n';

    // select whole body controller type
    wbc_type_ = util::ReadParameter<std::string>(interface_cfg,
                                                 "whole_body_controller");
    std::cout << "============================================" << '\n';
    std::cout << "WHOLE BODY CONTROLLER: " << wbc_type_ << '\n';
    std::cout << "============================================" << '\n';

    if (test_env_name == "mujoco") {
      if (wbc_type_ == "ihwbc")
        cfg_ =
            YAML::LoadFile(THIS_COM "config/optimo/sim/mujoco/ihwbc/pnc.yaml");
      else if (wbc_type_ == "wbic")
        cfg_ =
            YAML::LoadFile(THIS_COM "config/optimo/sim/mujoco/wbic/pnc.yaml");
    } else if (test_env_name == "pybullet") {
      if (wbc_type_ == "ihwbc")
        cfg_ = YAML::LoadFile(THIS_COM
                              "config/optimo/sim/pybullet/ihwbc/pnc.yaml");
      if (wbc_type_ == "wbic")
        cfg_ =
            YAML::LoadFile(THIS_COM "config/optimo/sim/pybullet/wbic/pnc.yaml");
    } else {
      assert(false);
    }

    // WBC controller frequency
    sp_->servo_dt_ = util::ReadParameter<double>(cfg_, "servo_dt");
    sp_->data_save_freq_ = util::ReadParameter<int>(cfg_, "data_save_freq");

#if B_USE_ZMQ
    // TODO:ZMQ socket communication
    // if (!OptimoDataManager::GetDataManager()->IsInitialized()) {
    // std::string socket_address =
    // util::ReadParameter<std::string>(cfg_, "ip_address");
    // OptimoDataManager::GetDataManager()->InitializeSocket(socket_address);
    //}
#endif

  } catch (const std::runtime_error &ex) {
    std::cerr << "Error Reading Parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }
}
void OptimoInterface::_SafeCommand(OptimoSensorData *data,
                                   OptimoCommand *command) {
  command->joint_pos_cmd_ = data->joint_pos_;
  command->joint_vel_cmd_.setZero();
  command->joint_trq_cmd_.setZero();
}

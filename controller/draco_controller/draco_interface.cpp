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

  // initialize robot model
  // robot_ =
  // new PinocchioRobotSystem(THIS_COM
  // "robot_model/draco/draco_modified.urdf", THIS_COM "robot_model/draco",
  // false, false);
  std::vector<std::string> unactuated_joint_list = {"l_knee_fe_jp",
                                                    "r_knee_fe_jp"};
  robot_ = new PinocchioRobotSystem(
      THIS_COM "robot_model/draco/draco_latest_collisions.urdf",
      THIS_COM "robot_model/draco", false, false, &unactuated_joint_list);

  // set locomotion control point
  robot_->SetFeetControlPoint("l_foot_contact", "r_foot_contact");

  // set control parameters
  this->SetParameters();

  // initialize state estimator
  if (state_estimator_type_ == "default")
    se_ = new DracoStateEstimator(robot_, cfg_);
  else if (state_estimator_type_ == "kf")
    se_ = new DracoKFStateEstimator(robot_, cfg_);
  else {
    std::cout
        << "[DracoInterface] Please check the state estimator type in pnc.yaml"
        << '\n';
    assert(false);
  }

  // initialize controller
  ctrl_arch_ = new DracoControlArchitecture(robot_, cfg_);
  interrupt_handler_ = new DracoInterruptHandler(
      static_cast<DracoControlArchitecture *>(ctrl_arch_));
  task_gain_handler_ = new DracoTaskGainHandler(
      static_cast<DracoControlArchitecture *>(ctrl_arch_));
}

DracoInterface::~DracoInterface() {
  delete robot_;
  delete se_;
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

  // estimate states
  if (b_cheater_mode_)
    se_->UpdateGroundTruthSensorData(draco_sensor_data);
  else {
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

  // step control count
  count_++;
}

void DracoInterface::_SafeCommand(DracoSensorData *data,
                                  DracoCommand *command) {
  command->joint_pos_cmd_ = data->joint_pos_;
  command->joint_vel_cmd_.setZero();
  command->joint_trq_cmd_.setZero();
}

void DracoInterface::SetParameters() {
  try {
    // select test environment
    YAML::Node interface_cfg =
        YAML::LoadFile(THIS_COM "config/draco/INTERFACE.yaml");
    std::string test_env_name =
        util::ReadParameter<std::string>(interface_cfg, "test_env_name");
    std::cout << "============================================" << '\n';
    std::cout << "TEST ENV: " << test_env_name << '\n';
    std::cout << "============================================" << '\n';

    if (test_env_name == "mujoco") {
      cfg_ = YAML::LoadFile(THIS_COM "config/draco/sim/mujoco/pnc.yaml");
    } else if (test_env_name == "pybullet") {
      cfg_ = YAML::LoadFile(THIS_COM "config/draco/sim/pybullet/pnc.yaml");
    } else if (test_env_name == "hw") {
      cfg_ = YAML::LoadFile(THIS_COM "config/draco/hw/pnc.yaml");
    } else {
      assert(false);
    }

    // WBC controller frequency
    sp_->servo_dt_ = util::ReadParameter<double>(cfg_, "servo_dt");
    sp_->data_save_freq_ = util::ReadParameter<int>(cfg_, "data_save_freq");

    // select state estimator
    state_estimator_type_ = util::ReadParameter<std::string>(
        cfg_["state_estimator"], "state_estimator_type");
    b_cheater_mode_ =
        util::ReadParameter<bool>(cfg_["state_estimator"], "b_cheater_mode");

    // select stance foot side
    int stance_foot = util::ReadParameter<int>(cfg_, "stance_foot");
    if (stance_foot == 0) {
      sp_->stance_foot_ = draco_link::l_foot_contact;
      sp_->prev_stance_foot_ = draco_link::l_foot_contact;
    } else if (stance_foot == 1) {
      sp_->stance_foot_ = draco_link::r_foot_contact;
      sp_->prev_stance_foot_ = draco_link::r_foot_contact;
    } else {
      assert(false);
    }

#if B_USE_ZMQ
    // ZMQ socket communication
    if (!DracoDataManager::GetDataManager()->IsInitialized()) {
      std::string socket_address =
          util::ReadParameter<std::string>(cfg_, "ip_address");
      DracoDataManager::GetDataManager()->InitializeSocket(socket_address);
    }
#endif

  } catch (const std::runtime_error &ex) {
    std::cerr << "Error Reading Parameter [" << ex.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
  }
}

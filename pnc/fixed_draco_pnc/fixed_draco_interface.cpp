#include "pnc/fixed_draco_pnc/fixed_draco_interface.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_control_architecture.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_estimator.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"

#include "pnc/robot_system/dart_robot_system.hpp"

//#include "pnc/fixed_draco_pnc/fixed_draco_data_manager.hpp"

#include "configuration.hpp"
#include "util/util.hpp"

FixedDracoInterface::FixedDracoInterface() : Interface() {

  robot_ = new DartRobotSystem(THIS_COM "robot_model/draco/draco_rel_path.urdf",
                               true, false);
  se_ = new FixedDracoStateEstimator(robot_);
  sp_ = FixedDracoStateProvider::GetStateProvider();

  // get yaml node
  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/fixed_draco/pnc.yaml");
  waiting_count_ = util::ReadParameter<int>(cfg["controller"], "waiting_count");

  control_architecture_ = new FixedDracoControlArchitecture(robot_);
  bool b_exp = util::ReadParameter<bool>(cfg, "b_exp");
  if (b_exp) {
    control_architecture_->state = FixedDracoState::kInitialize;
  } else {
    control_architecture_->state = FixedDracoState::kHold;
  }

  // initalize data publisher
  // FixedDracoDataManager::GetDataManager()->InitializeSocket(
  // util::ReadParameter<std::string>(cfg, "ip_address"));
}

FixedDracoInterface::~FixedDracoInterface() {
  delete robot_;
  delete se_;
  delete control_architecture_;
}

void FixedDracoInterface::GetCommand(void *_sensor_data, void *_command_data) {
  running_time_ = (double)(count_)*sp_->servo_dt;
  sp_->state = control_architecture_->state_;
  sp_->prev_state = control_architecture_->prev_state_;
  sp_->current_time = running_time_;

  FixedDracoSensorData *sensor_data = ((FixedDracoSensorData *)_sensor_data);
  FixedDracoCommand *command_data = ((FixedDracoCommand *)_command_data);

  if (count_ < waiting_count_) {
    // se_->InitializeModel(sensor_data);
    se_->UpdateModelWithGroundTruth(sensor_data);
    // this->GravityCompCommand(command_data);
    // this->InitialCommand(sensor_data, command_data);
    control_architecture_->GetCommand(command_data);
  } else {
    // robot model update using sensor_data
    se_->UpdateModelWithGroundTruth(sensor_data);
    // this->InitialCommand(sensor_data, command_data);
    //  se_->UpdateModel(sensor_data);
    //  get command from controller
    control_architecture_->GetCommand(command_data);
  }

  // FixedDracoDataManager::GetDataManager()->data_->time_ = sp_->current_time;
  // FixedDracoDataManager::GetDataManager()->SendData();
  ++count_;
}

void FixedDracoInterface::InitialCommand(FixedDracoSensorData *sensor_data,
                                         FixedDracoCommand *command) {
  for (auto it = sensor_data->joint_positions_.begin();
       it != sensor_data->joint_positions_.end(); it++) {
    command->joint_positions_cmd_[it->first] =
        sensor_data->joint_positions_[it->first];
    command->joint_velocities_cmd_[it->first] = 0.;
    command->joint_torques_cmd_[it->first] = 0.;
  }
}

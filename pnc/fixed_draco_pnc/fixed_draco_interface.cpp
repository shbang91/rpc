#include "pnc/fixed_draco_pnc/fixed_draco_interface.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_estimator.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_state_provider.hpp"
//#include "pnc/fixed_draco_pnc/fixed_draco_control_architecture.hpp"

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

  // control_architecture_ = new FixedDracoControlArchitecture(robot_);

  // initalize data publisher
  //FixedDracoDataManager::GetDataManager()->InitializeSocket(
      //util::ReadParameter<std::string>(cfg, "ip_address"));
}

FixedDracoInterface::~FixedDracoInterface() {
  delete robot_;
  delete se_;
  // delete control_architecture_;
}

void FixedDracoInterface::GetCommand(void *_sensor_data, void *_command_data) {

  running_time_ = (double)(count_)*sp_->servo_dt_;

  sp_->current_time_ = running_time_;

  FixedDracoSensorData *sensor_data = ((FixedDracoSensorData *)_sensor_data);
  FixedDracoCommand *command_data = ((FixedDracoCommand *)_command_data);

  int waiting_count = 10000000;
  if (count_ < waiting_count) {
    // se_->InitializeModel(sensor_data);
    se_->UpdateModelWithGroundTruth(sensor_data);
    //TEST
    //std::cout << "PNC robot ==========================================" << std::endl;
    //std::cout <<  robot_->GetLinkIso("r_foot_contact").translation().transpose() << std::endl;
    //TEST
    this->InitialCommand(sensor_data, command_data);
  } else {
    // robot model update using sensor_data
    se_->UpdateModelWithGroundTruth(sensor_data);
    //std::cout << "PNC robot ==========================================" << std::endl;
    //std::cout <<  robot_->GetLinkIso("r_foot_contact").translation().transpose() << std::endl;
    this->InitialCommand(sensor_data, command_data);
    // se_->UpdateModel(sensor_data);
    // get command from controller
    // control_architecture_->GetCommand(command_data);
  }

  //FixedDracoDataManager::GetDataManager()->data_->time_ = sp_->current_time_;
  //FixedDracoDataManager::GetDataManager()->SendData();
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

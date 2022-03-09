#include "pnc/draco_pnc/draco_interface.hpp"
#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/draco_pnc/draco_state_estimator.hpp"
//#include "pnc/draco_pnc/draco_control_architecture.hpp"

#include "pnc/robot_system/dart_robot_system.hpp"

#include "configuration.hpp"

DracoInterface::DracoInterface(): Interface() {

    robot_ = new DartRobotSystem(THIS_COM "robot_model/draco/draco_rel_path.urdf", false, false);
    se_ = new DracoStateEstimator(robot_);
    sp_ = DracoStateProvider::GetStateProvider();
    sp_->stance_foot_ = "l_foot_contact";
    sp_->prev_stance_foot_ = "l_foot_contact";
    //config setting

    //control_architecture_ = new DracoControlArchitecture(robot_);
}

DracoInterface::~DracoInterface(){
    delete robot_;
    delete se_;
    //delete control_architecture_;
}

void DracoInterface::GetCommand(void *_sensor_data, void *_command_data){
    DracoSensorData *sensor_data = ((DracoSensorData *)_sensor_data);
    DracoCommand *command_data = ((DracoCommand *)_command_data);

    //int waiting_count = 10;
    //if (count_ < waiting_count) {
    se_->InitializeModel(sensor_data); 
    this->InitialCommand(sensor_data, command_data);
       // TODO:initial config control
    //}
    //robot model update using sensor_data
    //se_->UpdateModel(sensor_data);
    //get command from controller
    //control_architecture_->GetCommand(command_data);

    //running_time_ = (double)(count_)*sp_->servo_dt;
    ++count_;
}

void DracoInterface::InitialCommand(DracoSensorData *sensor_data, DracoCommand *command){
    for(std::map<std::string, double>::iterator it = sensor_data->joint_positions_.begin();
       it != sensor_data->joint_positions_.end(); it++){
        command->joint_positions_cmd_[it->first] = sensor_data->joint_positions_[it->first];
        command->joint_velocities_cmd_[it->first] = 0.;
        command->joint_torques_cmd_[it->first] = 0.;
    }
}


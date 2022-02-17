#include "pnc/draco_pnc/draco_interface.hpp"
//#include "pnc/draco_pnc/draco_state_provider.hpp"
#include "pnc/draco_pnc/draco_state_estimator.hpp"
//#include "pnc/draco_pnc/draco_control_architecture.hpp"

#include "pnc/robot_system/dart_robot_system.hpp"

#include "configuration.hpp"

DracoInterface::DracoInterface(): Interface() {

    robot_ = new DartRobotSystem(THIS_COM "robot_model/draco/draco_rel_path.urdf", false, false);
    se_ = new DracoStateEstimator(robot_);
    //sp_ = new DracoStateProvider();

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

    int waiting_count = 10;
    if (count_ < waiting_cout) {
       se_->InitializeModel(sensor_data); 
       // TODO:initial config control
    }
    //robot model update using sensor_data
    se_->UpdateModel(sensor_data);
    //get command from controller
    //control_architecture_->GetCommand(command_data);

    //running_time_ = (double)(count_)*sp_->servo_dt;
    ++count_;
}

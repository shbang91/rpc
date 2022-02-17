#include "pnc/draco_pnc/draco_state_estimator"

DracoStateEstimator::DracoStateEstimator(RobotSystem *_robot){
    robot_ = _robot;

}

DracoStateEstimator::~DracoStateEstimator(){};

void DracoStateEstimator::InitializeModel(DracoSensorData *_sensor_data){
    this->UpdateModel(_sensor_data);

}

void DracoStateEstimator::UpdateModel(DracoSensorData *_sensor_data){

    //Estimate floating base states
    this->EstimateOrientation(_sensor_data);
    this->EstiamtePosition(_sensor_data);


    //Update robot model
    robot_->UpdateRobotModel(const Eigen::Vector3d &_base_com_pos,
        const Eigen::Quaternion<double> &_base_com_quat,
        const Eigen::Vector3d &_base_com_lin_vel,
        const Eigen::Vector3d &_base_com_ang_vel,
        const Eigen::Vector3d &_base_joint_pos,
        const Eigen::Quaternion<double> &_base_joint_quat,
        const Eigen::Vector3d &_base_joint_lin_vel,
        const Eigen::Vector3d &_base_joint_ang_vel,
        std::map<std::string, double> _joint_positions,
        std::map<std::string, double> _joint_velocities,
        const bool _b_update_centroid);


}

void DracoStateEstimator::EstimateOrientation(DracoSensorData* _sensor_data){}

void DracoStateEstimator::EstimatePosition(DracoSensorData* _sensor_data){}

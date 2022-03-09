#pragma once

#include "pnc/draco_pnc/draco_interface.hpp"
#include "pnc/robot_system/robot_system.hpp"
#include "pnc/draco_pnc/draco_state_provider.hpp"

class DracoStateEstimator{
    public:
        DracoStateEstimator(RobotSystem *_robot); 
        ~ DracoStateEstimator();

        void InitializeModel(DracoSensorData *_sensor_data);
        void UpdateModel(DracoSensorData *_sensor_data);

        void EstimateOrientation(DracoSensorData *_sensor_data);
        void EstimatePosition(DracoSensorData *_sensor_data);

        void ComputeDCM();

    private:
        RobotSystem *robot_;
        DracoStateProvider *sp_;

        Eigen::Vector3d base_joint_position_;
        Eigen::Matrix3d base_joint_orientation_;
        Eigen::Vector3d base_com_position_;

        Eigen::Vector3d prev_base_joint_position_;
        Eigen::Vector3d prev_base_com_position_;

        Eigen::Vector3d base_joint_lin_vel_;
        Eigen::Vector3d base_com_lin_vel_;

        Eigen::Vector3d global_leg_odometry_;


};

#pragma once

#include "pnc/fixed_draco_pnc/fixed_draco_interface.hpp"

class RobotSystem;
class FixedDracoStateProvider;

class FixedDracoStateEstimator{
    public:
        FixedDracoStateEstimator(RobotSystem *_robot); 
        ~ FixedDracoStateEstimator();

        void UpdateModelWithGroundTruth(FixedDracoSensorData *_sensor_data);
        void InitializeModel(FixedDracoSensorData *_sensor_data);
        void UpdateModel(FixedDracoSensorData *_sensor_data);

        void EstimateOrientation(FixedDracoSensorData *_sensor_data);


    private:
        RobotSystem *robot_;
        FixedDracoStateProvider *sp_;

        Eigen::Matrix3d base_joint_orientation_;
};

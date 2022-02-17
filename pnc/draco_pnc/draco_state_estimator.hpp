#pragma once

#include "pnc/draco_pnc/draco_interface.hpp"
#include "pnc/robot_system/robot_system.hpp"


class DracoStateEstimator{
    public:
        DracoStateEstimator(RobotSystem *_robot); 
        ~ DracoStateEstimator();

        void InitializeModel(DracoSensorData *_sensor_data);
        void UpdateModel(DracoSensorData *_sensor_data);

        void EstimateOrientation(DracoSensorData *_sensor_data);
        void EstimatePosition(DracoSensorData *_sensor_data);

    private:
        RobotSystem *robot_;



};

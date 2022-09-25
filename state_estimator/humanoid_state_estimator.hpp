#pragma once

#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "humanoid_sensor_data.hpp"

//TODO convert includes to forward declarations

class HumanoidStateEstimator{
public:
    HumanoidStateEstimator(RobotSystem *robot){robot_ = robot;};
    virtual ~HumanoidStateEstimator() = default;

    virtual void initialize(HumanoidSensorData *sensorData) = 0;
    virtual void update(HumanoidSensorData *sensorData) = 0;

protected:
    RobotSystem *robot_;

    bool b_first_visit_;
};

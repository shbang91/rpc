#pragma once

#include <map>

#include "pnc/interface.hpp"

class DracoStateProvider;
class DracoStateEstimator;

class DracoInterface: public Interface {
    public:
        DracoInterface();
        virtual ~DracoInterface();
        virtual void GetCommand(void *_sensor_data, void *_command_data);

    private:
        DracoStateProvider *sp_;
        DracoStateEstimator *se_;

};

class DracoSensorData {
    public:
        DracoSensorData() {
        imu_frame_isometry_.setIdentity();
        imu_frame_velocities_.setZero();
        b_lf_contact_ = false;
        b_rf_contact_ = false;
        };
        ~DracoSensorData() {};

        std::map<std::string, double> joint_positions_;
        std::map<std::string, double> joint_velocities_;
        Eigen::Matrix<double, 4, 4> imu_frame_isometry_;
        Eigen::Matrix<double, 6, 1> imu_frame_velocities_;

        bool b_lf_contact_;
        bool b_rf_contact_;

};

class DracoCommand {
    public:
        DracoCommand() {};
        ~DracoCommand() {}

        std::map<std::string, double> joint_positions_cmd_;
        std::map<std::string, double> joint_velocities_cmd_;
        std::map<std::string, double> joint_torques_cmd_;
};

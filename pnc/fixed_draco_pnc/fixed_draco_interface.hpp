#pragma once

#include <map>

#include "pnc/interface.hpp"

class FixedDracoStateProvider;
class FixedDracoStateEstimator;

class FixedDracoSensorData {
    public:
        FixedDracoSensorData() {
        imu_frame_isometry_.setIdentity();
        imu_frame_velocities_.setZero();

        base_com_pos_.setZero();
        base_com_quat_.setZero();
        base_com_lin_vel_.setZero();
        base_com_ang_vel_.setZero();

        base_joint_pos_.setZero();
        base_joint_quat_.setZero();
        base_joint_lin_vel_.setZero();
        base_joint_ang_vel_.setZero();
        };
        ~FixedDracoSensorData() {};

        Eigen::Matrix<double, 4, 4> imu_frame_isometry_;
        Eigen::Matrix<double, 6, 1> imu_frame_velocities_;

        std::map<std::string, double> joint_positions_;
        std::map<std::string, double> joint_velocities_;

        //Debug purpose
        Eigen::Vector3d base_com_pos_;
        Eigen::Vector4d base_com_quat_;
        Eigen::Vector3d base_com_lin_vel_;
        Eigen::Vector3d base_com_ang_vel_;

        Eigen::Vector3d base_joint_pos_;
        Eigen::Vector4d base_joint_quat_;
        Eigen::Vector3d base_joint_lin_vel_;
        Eigen::Vector3d base_joint_ang_vel_;
};

class FixedDracoCommand {
    public:
        FixedDracoCommand() {};
        ~FixedDracoCommand() {}

        std::map<std::string, double> joint_positions_cmd_;
        std::map<std::string, double> joint_velocities_cmd_;
        std::map<std::string, double> joint_torques_cmd_;
};

class FixedDracoInterface: public Interface {
    public:
        FixedDracoInterface();
        virtual ~FixedDracoInterface();
        virtual void GetCommand(void *_sensor_data, void *_command_data);
        void InitialCommand(FixedDracoSensorData *data, FixedDracoCommand *command);

    private:
        FixedDracoStateProvider *sp_;
        FixedDracoStateEstimator *se_;

};

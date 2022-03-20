#pragma once

#include <map>

#include "pnc/interface.hpp"

class DracoStateProvider;
class DracoStateEstimator;

class DracoSensorData {
    public:
        DracoSensorData() {
        imu_frame_isometry_.setIdentity();
        imu_frame_velocities_.setZero();
        b_lf_contact_ = false;
        b_rf_contact_ = false;

        base_com_pos_.setZero();
        base_com_quat_.setZero();
        base_com_lin_vel_.setZero();
        base_com_ang_vel_.setZero();

        base_joint_pos_.setZero();
        base_joint_quat_.setZero();
        base_joint_lin_vel_.setZero();
        base_joint_ang_vel_.setZero();
        };
        ~DracoSensorData() {};

        std::map<std::string, double> joint_positions_;
        std::map<std::string, double> joint_velocities_;
        Eigen::Matrix<double, 4, 4> imu_frame_isometry_;
        Eigen::Matrix<double, 6, 1> imu_frame_velocities_;

        bool b_lf_contact_;
        bool b_rf_contact_;

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

class DracoCommand {
    public:
        DracoCommand() {};
        ~DracoCommand() {}

        std::map<std::string, double> joint_positions_cmd_;
        std::map<std::string, double> joint_velocities_cmd_;
        std::map<std::string, double> joint_torques_cmd_;
};

class DracoInterface: public Interface {
    public:
        DracoInterface();
        virtual ~DracoInterface();
        virtual void GetCommand(void *_sensor_data, void *_command_data);
        void InitialCommand(DracoSensorData *data, DracoCommand *command);

    private:
        DracoStateProvider *sp_;
        DracoStateEstimator *se_;

};

#pragma once

#include <Eigen/Dense>
#include <zmq.hpp>
#include "draco.pb.h" //in build/messages/

class DracoData;


//Singleton class
class DracoDataManager {
    public:
        static DracoDataManager *GetDataManager();
        ~DracoDataManager();

        void InitializeSocket(const std::string &ip_address);
        void SendData();

        std::unique_ptr<DracoData> data_;

    private:
        DracoDataManager();

        std::unique_ptr<zmq::context_t> context_;
        std::unique_ptr<zmq::socket_t> socket_;

        bool b_initialize_socket_;

};

class DracoData {
    public:
        DracoData();
        ~DracoData();

        double time_;

        Eigen::Vector3d base_joint_pos_;
        Eigen::Vector4d base_joint_ori_;
        Eigen::Vector3d base_joint_lin_vel_;
        Eigen::Vector3d base_joint_ang_vel_;

        Eigen::Vector3d base_com_pos_;
        Eigen::Vector4d base_com_ori_;
        Eigen::Vector3d base_com_lin_vel_;
        Eigen::Vector3d base_com_ang_vel_;

};

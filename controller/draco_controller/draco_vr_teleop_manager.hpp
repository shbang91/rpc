#pragma once

#include "draco.pb.h"
#include "draco_definition.hpp"
#include <Eigen/Dense>
#include <Eigen/src/Core/Matrix.h>
#include <memory>
#include <zmq.hpp>

struct DracoVRCommands {
public: 
    DracoVRCommands(){};
    ~DracoVRCommands() = default;

    Eigen::Vector3d lh_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d rh_pos = Eigen::Vector3d::Zero();

    Eigen::Vector4d lh_ori = Eigen::Vector4d::Zero();
    Eigen::Vector4d rh_ori = Eigen::Vector4d::Zero();

    float l_trigger = 0;
    float l_bump = 0;
    float l_button = 0;
    float l_pad = 0;

    float r_trigger = 0;
    float r_bump = 0;
    float r_button = 0;
    float r_pad = 0;
};

// Singleton
class DracoVRTeleopManager {
public:
    static DracoVRTeleopManager *GetDataManager();
    ~DracoVRTeleopManager() = default;

    void InitializeTeleopSocket(const std::string &ip_address);
    DracoVRCommands *ReceiveCommands();

private:
    DracoVRTeleopManager();

    std::unique_ptr<zmq::context_t> context_;
    std::unique_ptr<zmq::socket_t> teleop_socket_;
    std::unique_ptr<zmq::socket_t> streaming_socket_;
};


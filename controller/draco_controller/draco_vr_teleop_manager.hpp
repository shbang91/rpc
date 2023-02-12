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

    Eigen::Matrix3d lh_ori = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d rh_ori = Eigen::Matrix3d::Zero();

    bool l_trigger = 0;
    bool l_bump = 0;
    bool l_button = 0;
    bool l_pad = 0;

    bool r_trigger = 0;
    bool r_bump = 0;
    bool r_button = 0;
    bool r_pad = 0;
};

// Singleton
class DracoVRTeleopManager {
public:
    static DracoVRTeleopManager *GetVRTeleopManager();
    ~DracoVRTeleopManager() = default;

    void InitializeTeleopSocket(const std::string &ip_address);
    DracoVRCommands ReceiveCommands();

private:
    DracoVRTeleopManager();

    std::unique_ptr<zmq::context_t> context_;
    std::unique_ptr<zmq::socket_t> teleop_socket_;
    std::unique_ptr<zmq::socket_t> streaming_socket_;
};


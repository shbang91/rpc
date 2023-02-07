#include <controller/draco_controller/draco_vr_teleop_manager.hpp>

DracoVRTeleopManager *DracoVRTeleopManager::GetDataManager() {
    static DracoVRTeleopManager vr;
    return &vr;
}

DracoVRTeleopManager::DracoVRTeleopManager() {
    // initialize zmq
    context_ = std::make_unique<zmq::context_t>(1);
    teleop_socket_ = std::make_unique<zmq::socket_t>(*context_, ZMQ_PULL);
    teleop_socket_->set(zmq::sockopt::conflate, true); // discards stale messages
}

void DracoVRTeleopManager::InitializeTeleopSocket(const std::string &ip_address) {
    teleop_socket_->connect(ip_address);
}

DracoVRCommands *DracoVRTeleopManager::ReceiveCommands() {
    zmq::message_t zmq_msg;
    //teleop_socket_.recv();
}

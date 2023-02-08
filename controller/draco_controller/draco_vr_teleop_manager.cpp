#include <chrono>
#include <controller/draco_controller/draco_vr_teleop_manager.hpp>
#include <zmq.hpp>

DracoVRTeleopManager *DracoVRTeleopManager::GetVRTeleopManager() {
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

DracoVRCommands DracoVRTeleopManager::ReceiveCommands() {
    zmq::message_t commands;
    auto start = std::chrono::high_resolution_clock::now();
    zmq::recv_result_t bytes_received = teleop_socket_->recv(commands, zmq::recv_flags::dontwait);
    std::cout << bytes_received.value_or(5555) << std::endl;
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << "Time taken by ZMQ PULL: " << duration.count() << std::endl;
    std::string commands_str(static_cast<char*>(commands.data()), commands.size());

    draco::vr_teleop_msg m;
    m.ParseFromString(commands_str);

    DracoVRCommands result;

    result.l_bump = m.l_bump();
    result.l_trigger = m.l_trigger();
    result.l_button = m.l_button();
    result.l_pad = m.l_pad();

    result.r_bump = m.r_bump();
    result.r_trigger = m.r_trigger();
    result.r_button = m.r_button();
    result.r_pad = m.r_pad();

    return result;
}

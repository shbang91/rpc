#include <chrono>
#include <controller/draco_controller/draco_vr_teleop_manager.hpp>
#include <google/protobuf/text_format.h>
#include <iostream>
#include <zmq.hpp>

DracoVRTeleopManager *DracoVRTeleopManager::GetVRTeleopManager() {
  static DracoVRTeleopManager vr;
  return &vr;
}

DracoVRTeleopManager::DracoVRTeleopManager() {
  // initialize zmq
  context_ = std::make_unique<zmq::context_t>(1);
  teleop_socket_ = std::make_unique<zmq::socket_t>(*context_, ZMQ_SUB);
  teleop_socket_->setsockopt(ZMQ_CONFLATE, 1); // discards stale messages
  char empty[0];
  teleop_socket_->setsockopt(ZMQ_SUBSCRIBE, empty); // accept all topics
  // note that "" doesn't work since sizeof("") = 1.
}

void DracoVRTeleopManager::InitializeTeleopSocket(
    const std::string &ip_address) {
  std::cout << "ip address " + ip_address << std::endl;
  teleop_socket_->connect(ip_address);
  teleop_socket_->setsockopt(ZMQ_RCVTIMEO, 2);
}

bool DracoVRTeleopManager::isReady() { return ready_; }

DracoVRCommands DracoVRTeleopManager::ReceiveCommands() {
  zmq::message_t commands;
  // auto start = std::chrono::high_resolution_clock::now();
  bool success = teleop_socket_->recv(&commands);
  // auto stop = std::chrono::high_resolution_clock::now();
  // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop
  // - start); std::cout << "Time taken by ZMQ PULL: " << duration.count() <<
  // std::endl;
  DracoVRCommands result;
  if (!success) {
    // std::cout << "No commands received" << std::endl;
    return prev_commands;
  }

  draco::vr_teleop_msg m;
  m.ParseFromArray(commands.data(), commands.size());

  /*
  std::string s;
  google::protobuf::TextFormat::PrintToString(m, &s);
  std::cout << s << std::endl;
  */

  result.l_bump = m.l_bump();
  result.l_trigger = m.l_trigger();
  result.l_button = m.l_button();
  result.l_pad = m.l_pad();

  result.r_bump = m.r_bump();
  result.r_trigger = m.r_trigger();
  result.r_button = m.r_button();
  result.r_pad = m.r_pad();

  for (int i = 0; i < 3; ++i) {
    result.lh_pos[i] = m.lh_pos(i);
    result.rh_pos[i] = m.rh_pos(i);
    for (int j = 0; j < 3; ++j) {
      result.lh_ori(i, j) = m.lh_ori(i * 3 + j);
      result.rh_ori(i, j) = m.rh_ori(i * 3 + j);
    }
  }

  if (result.l_pad) {
    std::cout << "l pad pressed" << result.l_pad << std::endl;
    if (!l_pad_held_) {
      ready_ = !ready_;
      std::cout << "VR toggled" << std::endl;
    }
    l_pad_held_ = true;
  } else {
    l_pad_held_ = false;
  }

  prev_commands = result;
  // If the next command doesn't get received,
  // we want to replay the previous command.
  // However, we don't want to toggle vr ready again
  prev_commands.l_pad = false;

  return result;
}

#pragma once
#include <iostream>
#include <memory>
#include <zmq.hpp>

class TeleopHandler {
public:
  TeleopHandler() : b_teleop_ready_(false) {
    context_ = std::make_unique<zmq::context_t>(1);
    teleop_socket_ = std::make_unique<zmq::socket_t>(*context_, ZMQ_SUB);
    teleop_socket_->setsockopt(ZMQ_CONFLATE, 1); // discards stale msg
    char empty[0]; // noe that "" doesn't work since sizeof("") =1
    teleop_socket_->setsockopt(ZMQ_SUBSCRIBE, empty); // accept all topics
  };
  virtual ~TeleopHandler() = default;

  bool InitializeSocket(const std::string &ip_address) {
    std::cout << "teleop ip address: " << ip_address << '\n';
    teleop_socket_->connect(ip_address);
    teleop_socket_->setsockopt(ZMQ_RCVTIMEO, 2);
    return true;
  };
  virtual void ReceiveCommands(void *command) = 0;

  // getter
  bool IsReady() const { return b_teleop_ready_; }

protected:
  std::unique_ptr<zmq::context_t> context_;
  std::unique_ptr<zmq::socket_t> teleop_socket_;

  bool b_teleop_ready_;
};

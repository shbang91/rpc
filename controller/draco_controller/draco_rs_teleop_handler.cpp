#include "controller/draco_controller/draco_rs_teleop_handler.hpp"
#include "util/util.hpp"

#include "draco.pb.h"

DracoRSTeleopHandler::DracoRSTeleopHandler() : TeleopHandler() {
  util::PrettyConstructor(3, "DracoRealsenseTeleopHandler");
}

void DracoRSTeleopHandler::ReceiveCommands(void *command) {
  DracoRSCommands *rs_command = static_cast<DracoRSCommands *>(command);
  // clock_->Start();
  zmq::message_t msg;
  bool success = teleop_socket_->recv(&msg);
  // clock_->Stop();
  // std::cout << "Time taken by ZMQ pull: " << clock_->duration() << " ms"
  //<< '\n';

  if (!success) {
    std::cout << "No commands through ZMQ received!" << std::endl;
    // use prev command
    rs_command->pos_ = prev_command_.pos_;
    rs_command->quat_ = prev_command_.quat_;
    rs_command->b_grasp_ = prev_command_.b_grasp_;
    rs_command->b_teleop_toggled_ = prev_command_.b_teleop_toggled_;
  } else {
    draco::rs_teleop_msg m;
    m.ParseFromArray(msg.data(), msg.size());

    // position
    for (int i = 0; i < 3; ++i) {
      rs_command->pos_[i] = m.pos(i);
    }
    // orientation
    rs_command->quat_.x() = m.quat(0);
    rs_command->quat_.y() = m.quat(1);
    rs_command->quat_.z() = m.quat(2);
    rs_command->quat_.w() = m.quat(3);

    rs_command->b_grasp_ = m.b_grasp();
    rs_command->b_teleop_toggled_ = m.b_teleop_toggled();

    // if (rs_command->b_teleop_toggled_) {
    // std::cout << "Teleop Toggled!" << std::endl;
    // if (l_pad_held_ == false) {
    // b_teleop_ready_ = true;
    // std::cout << "Teleop Ready!" << std::endl;
    //}
    // l_pad_held_ = true;
    //} else {
    // l_pad_held_ = false;
    //}

    // update prev command
    prev_command_.pos_ = rs_command->pos_;
    prev_command_.quat_ = rs_command->quat_;
    prev_command_.b_grasp_ = rs_command->b_grasp_;
    prev_command_.b_teleop_toggled_ = rs_command->b_teleop_toggled_;
    // If the next command doesn't get received,
    // We want to replay the previous command.
    // However, we don't wnat to toggle teleop ready again
    // prev_command_.b_teleop_toggled_ = false;
  }

  b_teleop_ready_ = rs_command->b_teleop_toggled_;
}

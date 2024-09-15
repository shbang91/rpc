#pragma once
#include <Eigen/Dense>

#include "controller/teleop_handler.hpp"
#include "util/clock.hpp"

struct DracoRSCommands {
  DracoRSCommands() {
    pos_ = Eigen::Vector3d::Zero();
    quat_ = Eigen::Quaterniond::Identity();
    b_grasp_ = false;
    b_teleop_toggled_ = false;
  };
  ~DracoRSCommands() = default;

  Eigen::Vector3d pos_;
  Eigen::Quaterniond quat_;
  bool b_grasp_;
  bool b_teleop_toggled_;
};

// Draco Teleop handler using Realsense T265
class DracoRSTeleopHandler : public TeleopHandler {
public:
  DracoRSTeleopHandler();
  virtual ~DracoRSTeleopHandler() = default;

  void ReceiveCommands(void *command) override;

protected:
  bool l_pad_held_;
  Clock clock_;
  DracoRSCommands prev_command_;
};

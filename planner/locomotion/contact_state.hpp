#pragma once
#include <Eigen/Dense>

class ContactState {
public:
  ContactState()
      : time_ini_(0.), time_end_(0.), position_(Eigen::Vector3d::Zero()),
        orientation_(Eigen::Quaterniond::Identity()) {}
  virtual ~ContactState() = default;

  // getter
  double GetContactActivationTime() { return time_ini_; }
  double GetContactDeactivationTime() { return time_end_; }
  Eigen::Vector3d GetContactPosition() { return position_; }
  Eigen::Quaterniond GetContactOrientation() { return orientation_; }

private:
  double time_ini_;
  double time_end_;
  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;
};

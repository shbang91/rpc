#pragma once

#include <Eigen/Dense>

template <typename T> class CubicBeizerTrajectoryManager {
public:
  CubicBeizerTrajectoryManager() {
    p0_.setZero();
    pf_.setZero();
    p_.setZero();
    v_.setZero();
    a_.setZero();
    height_ = T(0.0);
  }

  void ComputeSwingTrajectoryBezier(T phase, T swing_time);
  void ComputeSwingTrajectoryBezier(T phase);

  // setter
  void SetInitialPosition(const Eigen::Matrix<T, 3, 1> &p0) { p0_ = p0; }
  void SetFinalPosition(const Eigen::Matrix<T, 3, 1> &pf) { pf_ = pf; }
  void SetHeight(const T h) { height_ = h; }

  // getter
  Eigen::Matrix<T, 3, 1> GetPosition() { return p_; }
  Eigen::Matrix<T, 3, 1> GetVelocity() { return v_; }
  Eigen::Matrix<T, 3, 1> GetAcceleration() { return a_; }

  Eigen::Matrix<T, 3, 1> GetFinalPosition() { return pf_; }
  Eigen::Matrix<T, 3, 1> GetInitialPosition() { return p0_; }

private:
  Eigen::Matrix<T, 3, 1> p0_, pf_, p_, v_, a_;
  T height_;
};

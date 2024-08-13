#pragma once
#include "util/util.hpp"
#include <Eigen/Dense>
#include <cassert>
#include <iostream>
#include <vector>

// Assume first 6 joints are for floating base if the robot has a floating base
class WBC {
public:
  WBC(const std::vector<bool> &act_qdot_list)
      : num_qdot_(0), num_active_(0), num_passive_(0), dim_contact_(0),
        b_update_setting_(false), b_contact_(true) {

    num_qdot_ = act_qdot_list.size();

    M_ = Eigen::MatrixXd::Zero(num_qdot_, num_qdot_);
    Minv_ = Eigen::MatrixXd::Zero(num_qdot_, num_qdot_);
    cori_ = Eigen::VectorXd::Zero(num_qdot_);
    grav_ = Eigen::VectorXd::Zero(num_qdot_);

    // check if the robot has a floating base
    if (act_qdot_list[0]) {
      num_floating_ = 0;
      b_floating_base_ = false;
    } else {
      num_floating_ = 6;
      b_floating_base_ = true;
    }

    // check the number of active and passive joints
    for (int i(0); i < act_qdot_list.size(); ++i) {
      if (act_qdot_list[i])
        ++num_active_;
      if (!b_floating_base_) {
        // non floating base robot
        if (!act_qdot_list[i])
          ++num_passive_;
      } else {
        // floating base robot
        if (!act_qdot_list[i] && i >= num_floating_)
          ++num_passive_;
      }
    }

    sa_.setZero(num_active_, num_qdot_);
    sf_.setZero(num_floating_, num_qdot_);
    sv_.setZero(num_passive_, num_qdot_);
    snf_.setZero(num_active_ + num_passive_, num_qdot_);
    int j(0), k(0), e(0), l(0);
    for (int i(0); i < act_qdot_list.size(); i++) {
      if (act_qdot_list[i]) {
        sa_(j, i) = 1.;
        ++j;
      } else {
        if (i < num_floating_) {
          sf_(e, i) = 1.;
          ++e;
        } else {
          sv_(k, i) = 1.;
          ++k;
        }
      }
      if (i >= num_floating_) {
        snf_(l, i) = 1.;
        ++l;
      }
    }

    assert(num_qdot_ - num_active_ - num_passive_ - num_floating_ == 0);

    // internal constraint check
    if (num_passive_ == 0)
      b_internal_constraint_ = false;
    else
      b_internal_constraint_ = true;
  }

  virtual ~WBC() = default;

  void UpdateSetting(const Eigen::MatrixXd &M, const Eigen::MatrixXd &Minv,
                     const Eigen::VectorXd &cori, const Eigen::VectorXd &grav) {
    M_ = M;
    Minv_ = Minv;
    cori_ = cori;
    grav_ = grav;
    b_update_setting_ = true;
  }

  virtual void SetParameters(const YAML::Node &node) = 0;

protected:
  int num_qdot_;
  int num_active_;
  int num_passive_;
  int num_floating_;
  int dim_contact_;
  bool b_internal_constraint_;
  bool b_floating_base_;
  bool b_contact_;

  Eigen::MatrixXd sf_;
  Eigen::MatrixXd sa_;
  Eigen::MatrixXd sv_;
  Eigen::MatrixXd snf_;

  Eigen::MatrixXd M_;
  Eigen::MatrixXd Minv_;
  Eigen::VectorXd cori_;
  Eigen::VectorXd grav_;
  bool b_update_setting_;
};

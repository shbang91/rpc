#pragma once

#include "controller/draco_controller/draco_crbi/draco3_crbi_helper.h"
#include "controller/models/composite_rigid_body_inertia.hpp"

class DracoCompositeRigidBodyInertia : public CompositeRigidBodyInertia {
public:
  DracoCompositeRigidBodyInertia();
  virtual ~DracoCompositeRigidBodyInertia();

  // Ori consists of euler_x, euler_y, euler_z
  Eigen::Matrix3d ComputeInertia(const Eigen::Vector3d &base_pos,
                                 const Eigen::Vector3d &base_ori,
                                 const Eigen::Vector3d &lfoot_pos,
                                 const Eigen::Vector3d &lfoot_ori,
                                 const Eigen::Vector3d &rfoot_pos,
                                 const Eigen::Vector3d &rfoot_ori) override;

  //  Eigen::MatrixXd
  // ComputeDerivativeWrtInput(const Eigen::VectorXd &base_pose,
  // const Eigen::VectorXd &lfoot_pose,
  // const Eigen::VectorXd &rfoot_pose) override;

private:
  casadi_int f_sz_arg_;
  casadi_int f_sz_res_;
  casadi_int f_sz_iw_;
  casadi_int f_sz_w_;

  double **f_x_;
  double **f_y_;

  Eigen::MatrixXd f_input_ph_;
  Eigen::MatrixXd f_output_ph_;
};

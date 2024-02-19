#pragma once

#include "controller/whole_body_controller/task.hpp"
#include "util/util.hpp"

#include "controller/draco_controller/draco_task/draco_wbo_task_helper.h"

class PinocchioRobotSystem;
class DracoStateProvider;

class DracoWBOTask : public Task {
public:
  DracoWBOTask(PinocchioRobotSystem *robot);
  ~DracoWBOTask();

  void UpdateOpCommand() override;
  void UpdateOpCommand(const Eigen::Matrix3d &rot_world_local) override;
  void UpdateJacobian() override;
  void UpdateJacobianDotQdot() override;

  Eigen::Vector3d local_wbo_ang_vel_est_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d local_wbo_ang_vel_gt_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d centroidal_ang_mom_est_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d centroidal_ang_mom_gt_ = Eigen::Vector3d::Zero();

private:
  DracoStateProvider *sp_;

  Eigen::Vector3d _LocalWboQuatXYZ(const Eigen::VectorXd &joint_pos);
  Eigen::MatrixXd _WboQuatXYZJacobian(const Eigen::VectorXd &joint_pos);
  void _CopyEigenMatrixToDoubleArray(const Eigen::MatrixXd &mat,
                                     double **array_2d);
  void _CopyEigenMatrixToDoubleArray(const Eigen::MatrixXd &mat1,
                                     const Eigen::MatrixXd &mat2,
                                     double **array_2d);
  void _CopyEigenVectorToDoubleArray(const Eigen::VectorXd &vec,
                                     double **array_2d);
  void _CopyDoubleArrayToEigenMatrix(double **array_2d, Eigen::MatrixXd &mat,
                                     const int block_row, const int block_col);

  // misc
  Eigen::Matrix<double, 3, 4> _QuatRateToAngVel(const Eigen::Quaterniond &quat);
  Eigen::Matrix<double, 4, 3> _QuatMiscFunc(const Eigen::Quaterniond &quat);

  Eigen::VectorXd _GetJointPosReduced();
  Eigen::VectorXd _GetJointVelReduced();

  double **f_input_;
  double **f_output_;
  double **jac_f_input_;
  double **jac_f_output_;

  casadi_int f_sz_arg_;
  casadi_int f_sz_res_;
  casadi_int f_sz_iw_;
  casadi_int f_sz_w_;

  casadi_int jac_f_sz_arg_;
  casadi_int jac_f_sz_res_;
  casadi_int jac_f_sz_iw_;
  casadi_int jac_f_sz_w_;

  int num_input_;
  int dim_per_input_;
  int num_output_;
  int dim_per_output_;

  std::vector<int> actuated_joint_idx_;
  std::vector<int> ignored_joint_idx_;

  Eigen::Matrix3d world_R_base_ = Eigen::Matrix3d::Identity();
  Eigen::MatrixXd wbo_local_jacobian_;
};

#include "controller/draco_controller/draco_crbi/draco_composite_rigid_body_inertia.hpp"
#include "util/util.hpp"

DracoCompositeRigidBodyInertia::DracoCompositeRigidBodyInertia()
    : CompositeRigidBodyInertia(draco3_crbi_helper_n_in(), 3) {
  util::PrettyConstructor(2, "DracoCompositeRigidBodyInertia");

  draco3_crbi_helper_work(&f_sz_arg_, &f_sz_res_, &f_sz_iw_, &f_sz_w_);

  f_x_ = new double *[num_input_];
  for (int i = 0; i < num_input_; ++i)
    f_x_[i] = new double[dim_per_input_];

  f_y_ = new double *[num_output_];
  for (int i = 0; i < num_output_; ++i)
    f_y_[i] = new double[dim_per_output_];

  f_input_ph_ = Eigen::MatrixXd::Zero(num_input_, dim_per_input_);
  f_output_ph_ = Eigen::MatrixXd::Zero(num_output_, dim_per_output_);
}

DracoCompositeRigidBodyInertia::~DracoCompositeRigidBodyInertia() {
  for (int i = 0; i < num_input_; ++i)
    delete[] f_x_[i];
  delete[] f_x_;

  for (int i = 0; i < num_output_; ++i)
    delete[] f_y_[i];
  delete[] f_y_;
}

Eigen::Matrix3d DracoCompositeRigidBodyInertia::ComputeInertia(
    const Eigen::Vector3d &base_pos, const Eigen::Vector3d &base_ori,
    const Eigen::Vector3d &lfoot_pos, const Eigen::Vector3d &lfoot_ori,
    const Eigen::Vector3d &rfoot_pos, const Eigen::Vector3d &rfoot_ori) {

  for (int i = 0; i < dim_per_input_; ++i) {
    f_input_ph_(0, i) = base_pos[i];
    f_input_ph_(1, i) = base_ori[i];
    f_input_ph_(2, i) = lfoot_pos[i];
    f_input_ph_(3, i) = lfoot_ori[i];
    f_input_ph_(4, i) = rfoot_pos[i];
    f_input_ph_(5, i) = rfoot_ori[i];
  }

  _fill_double_array(f_input_ph_, f_x_);

  casadi_int f_iw[f_sz_iw_];
  double f_w[f_sz_w_];
  draco3_crbi_helper(const_cast<const double **>(f_x_), f_y_, f_iw, f_w,
                     draco3_crbi_helper_checkout());
  _fill_matrix(f_y_, f_output_ph_, num_output_, dim_per_output_);

  Eigen::Matrix3d ret = Eigen::Matrix3d::Zero();
  ret(0, 0) = f_output_ph_(0, 0); // I_xx
  ret(1, 1) = f_output_ph_(0, 1); // I_yy
  ret(2, 2) = f_output_ph_(0, 2); // I_zz

  ret(0, 1) = f_output_ph_(0, 3); // I_xy
  ret(1, 0) = f_output_ph_(0, 3); // I_yx
  ret(0, 2) = f_output_ph_(0, 4); // I_xz
  ret(2, 0) = f_output_ph_(0, 4); // I_zx
  ret(1, 2) = f_output_ph_(0, 5); // I_yz
  ret(2, 1) = f_output_ph_(0, 5); // I_zy

  return ret;
}

// Eigen::MatrixXd DracoCompositeRigidBodyInertia::ComputeDerivativeWrtInput(
// const Eigen::VectorXd &base_pose, const Eigen::VectorXd &lfoot_pose,
// const Eigen::VectorXd &rfoot_pose) {}

#pragma once
#include <Eigen/Dense>

class CompositeRigidBodyInertia {
public:
  CompositeRigidBodyInertia(int num_input, int dim_per_input);
  virtual ~CompositeRigidBodyInertia();

  // Ori consists of euler_x, euler_y, euler_z
  virtual Eigen::Matrix3d ComputeInertia(const Eigen::Vector3d &base_pos,
                                         const Eigen::Vector3d &base_ori,
                                         const Eigen::Vector3d &lfoot_pos,
                                         const Eigen::Vector3d &lfoot_ori,
                                         const Eigen::Vector3d &rfoot_pos,
                                         const Eigen::Vector3d &rfoot_ori) = 0;

  // virtual Eigen::MatrixXd
  // ComputeDerivativeWrtInput(const Eigen::VectorXd &base_pose,
  // const Eigen::VectorXd &lfoot_pose,
  // const Eigen::VectorXd &rfoot_pose) = 0;

protected:
  // Create 3x3 inertia matrix
  Eigen::MatrixXd _inertia_from_one_hot_vector(const Eigen::VectorXd &vec);
  // Use this for filling function args
  void _fill_double_array(const Eigen::MatrixXd &mat, double **x);
  // Use this for filling jacobian function args
  void _fill_double_array(const Eigen::MatrixXd &mat1,
                          const Eigen::MatrixXd &mat2, double **x);
  // Use this for parsing output
  // Assume x[1][mat.cols()*mat.rows()]
  void _fill_matrix(double **x, Eigen::MatrixXd &mat, int block_row,
                    int block_col);

  int num_input_;
  int dim_per_input_;
  int num_output_;
  int dim_per_output_;
};

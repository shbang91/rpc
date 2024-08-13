#pragma once
#include <Eigen/Dense>
#include <unordered_map>

#include "controller/whole_body_controller/wbc.hpp"

#include "third_party/optimizer/goldfarb/QuadProg++.hh"

class Task;
class Contact;
class InternalConstraint;
class ForceTask;
class IHWBC : public WBC {
public:
  IHWBC(const std::vector<bool> &act_qdot_list);
  virtual ~IHWBC() = default;

  void Solve(const std::unordered_map<std::string, Task *> &task_map,
             const std::map<std::string, Contact *> &contact_map,
             const std::unordered_map<std::string, InternalConstraint *>
                 &internal_constraint_map,
             std::map<std::string, ForceTask *> &force_task_map,
             Eigen::VectorXd &qddot_cmd, Eigen::VectorXd &trq_cmd);

  void ComputeTaskCosts(
      const std::unordered_map<std::string, Task *> &task_map,
      const std::map<std::string, ForceTask *> &force_task_map,
      std::unordered_map<std::string, double> &task_unweighted_cost_map,
      std::unordered_map<std::string, double> &task_weighted_cost_map);

  void SetParameters(const YAML::Node &node) override;

  // getter
  bool IsTrqLimit() const { return b_trq_limit_; }
  Eigen::VectorXd GetLambdaRf() const { return lambda_rf_; }
  double GetLambdaQddot() const { return lambda_qddot_; }

  // setter
  void SetTrqLimit(const Eigen::Matrix<double, Eigen::Dynamic, 2> &trq_limit) {
    joint_trq_limits_ = trq_limit;
  }

protected:
  void _SetQPCost(const Eigen::MatrixXd &cost_mat,
                  const Eigen::VectorXd &cost_vec);
  void _SetQPEqualityConstraint(const Eigen::MatrixXd &eq_mat,
                                const Eigen::VectorXd &eq_vec);
  void _SetQPInEqualityConstraint(const Eigen::MatrixXd &ineq_mat,
                                  const Eigen::VectorXd &ineq_vec);
  void _SolveQP();

  // ihwbc solve method firstvisit
  bool b_first_visit_;

  int dim_cone_constraint_;

  // optimization regularization parameters
  double lambda_qddot_;
  Eigen::VectorXd lambda_rf_;
  bool b_trq_limit_;

  // joint pos, vel, trq limits (cols(1) = lower limit / cols(2) = upper limit)
  Eigen::Matrix<double, Eigen::Dynamic, 2> joint_trq_limits_;

  /// QP solver interface
  int num_qp_vars_ = 1;
  int num_eq_const_ = 0;
  int num_ineq_const_ = 0;

  Eigen::VectorXd qddot_sol_;
  Eigen::VectorXd rf_sol_;
  Eigen::VectorXd trq_cmd_;

  /// QP decision variables.
  GolDIdnani::GVect<double> x_;
  // Eigen::VectorXd x_;

  /// QP cost.
  GolDIdnani::GMatr<double> G_;
  GolDIdnani::GVect<double> g0_;

  /// QP equality cosntraint.
  GolDIdnani::GMatr<double> CE_;
  GolDIdnani::GVect<double> ce0_;

  /// QP inequality constraint.
  GolDIdnani::GMatr<double> CI_;
  GolDIdnani::GVect<double> ci0_;
};

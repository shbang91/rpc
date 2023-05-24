#pragma once

#include <Eigen/Dense>
#include <map>
#include <string>

#include "controller/whole_body_controller/wbc.hpp"

#include "third_party/optimizer/goldfarb/QuadProg++.hh"

class Task;
class Contact;
class InternalConstraint;
class ForceTask;

struct QPParams {
  QPParams(int num_float, int dim_contact) {
    W_delta_qddot_ = Eigen::VectorXd::Zero(num_float);
    W_delta_rf_ = Eigen::VectorXd::Zero(dim_contact);
    W_xc_ddot_ = Eigen::VectorXd::Zero(dim_contact);
  }
  ~QPParams() = default;

  Eigen::VectorXd W_delta_qddot_;
  Eigen::VectorXd W_delta_rf_;
  Eigen::VectorXd W_xc_ddot_;
};

struct WBICData {
  WBICData(int num_float, int num_qdot, QPParams *qp_params) {
    qp_params_ = qp_params;
    delta_qddot_ = Eigen::VectorXd::Zero(num_float);
    delta_rf_ = Eigen::VectorXd::Zero(qp_params_->W_delta_rf_.size());
    corrected_wbc_qddot_cmd_ = Eigen::VectorXd::Zero(num_qdot);
    rf_cmd_ = Eigen::VectorXd::Zero(qp_params_->W_delta_rf_.size());
    Xc_ddot_ = Eigen::VectorXd::Zero(qp_params_->W_delta_rf_.size());
    delta_qddot_cost_ = 0;
    delta_rf_cost_ = 0;
    Xc_ddot_cost_ = 0;
  };
  ~WBICData() = default;

  // QP input
  QPParams *qp_params_;

  // QP Decision Variables
  Eigen::VectorXd delta_qddot_;
  Eigen::VectorXd delta_rf_;

  // QP optimal cost
  double delta_qddot_cost_;
  double delta_rf_cost_;
  double Xc_ddot_cost_;

  // WBIC result
  Eigen::VectorXd corrected_wbc_qddot_cmd_;
  Eigen::VectorXd rf_cmd_;
  Eigen::VectorXd Xc_ddot_;
};

class WBIC : public WBC {
public:
  WBIC(const std::vector<bool> &act_qdot_list,
       const Eigen::MatrixXd *Ji = nullptr);
  ~WBIC() = default;

  // Compute joint commands (jpos, jvel, jacc) using nullspace projection method
  bool FindConfiguration(const Eigen::VectorXd &curr_jpos,
                         const std::vector<Task *> &task_vector,
                         const std::vector<Contact *> &contact_vector,
                         Eigen::VectorXd &jpos_cmd, Eigen::VectorXd &jvel_cmd,
                         Eigen::VectorXd &wbc_qddot_cmd);

  // Compute joint torque using QP
  bool MakeTorque(const Eigen::VectorXd &wbc_qddot_cmd,
                  const std::vector<ForceTask *> &force_task_vector,
                  const std::map<std::string, Contact *> &contact_map,
                  Eigen::VectorXd &jtrq_cmd, WBICData *wbic_data);

private:
  void _PseudoInverse(const Eigen::MatrixXd &jac, Eigen::MatrixXd &jac_inv);
  void _WeightedPseudoInverse(const Eigen::MatrixXd &jac,
                              const Eigen::MatrixXd &W,
                              Eigen::MatrixXd &jac_bar);
  void _BuildProjectionMatrix(const Eigen::MatrixXd &jac, Eigen::MatrixXd &N,
                              const Eigen::MatrixXd *W = nullptr);

  // TODO: need to change if other contact involve (hands) for sequence
  void
  _BuildContactMtxVect(const std::map<std::string, Contact *> &contact_map);
  void
  _GetDesiredReactionForce(const std::vector<ForceTask *> &force_task_vector);
  void _SetQPCost(const Eigen::VectorXd &wbc_qddot_cmd);
  void _SetQPEqualityConstraint(const Eigen::VectorXd &wbc_qddot_cmd);
  void _SetQPInEqualityConstraint();
  void _SolveQP(const Eigen::VectorXd &wbc_qddot_cmd);
  void _GetSolution(Eigen::VectorXd &jtrq_cmd);

  double threshold_;

  // internal constraint nullspace
  Eigen::MatrixXd Ni_dyn_;

  // contact
  Eigen::MatrixXd Jc_;
  Eigen::VectorXd JcDotQdot_;
  Eigen::MatrixXd Uf_mat_;
  Eigen::MatrixXd Uf_vec_;
  Eigen::MatrixXd contact_rot_;
  int dim_contact_;

  // reaction force
  Eigen::VectorXd des_rf_;

  // WBIC data
  WBICData *wbic_data_;

  //=======================================================================
  // ProxQP
  //=======================================================================
  /*
   min 0.5 * x H x + g.T x
   s.t.
        Ax = b
        l <= Cx <= u
  */
  // cost
  Eigen::MatrixXd H_;
  Eigen::VectorXd g_;
  // equality
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  // inequality
  Eigen::MatrixXd C_;
  Eigen::VectorXd l_;
  // Eigen::VectorXd u_; // no upper bound for now

  //=======================================================================
  // QuadProg
  //=======================================================================
  // decision variables
  GolDIdnani::GVect<double> x_;

  // QP cost
  GolDIdnani::GMatr<double> G_;
  GolDIdnani::GVect<double> g0_;

  // QP equality constraint
  GolDIdnani::GMatr<double> CE_;
  GolDIdnani::GVect<double> ce0_;

  // QP inequality constraint
  GolDIdnani::GMatr<double> CI_;
  GolDIdnani::GVect<double> ci0_;
};

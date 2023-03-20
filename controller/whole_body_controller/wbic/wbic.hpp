#pragma once

#include <Eigen/Dense>
#include <map>
#include <string>

#include "controller/whole_body_controller/wbc.hpp"

class Task;
class Contact;
class InternalConstraint;
class ForceTask;

class WBICData {
public:
  WBICData(){};
  ~WBICData() = default;

  // input
  Eigen::VectorXd W_delta_qddot_;
  Eigen::VectorXd W_delta_rf_;

  // output
  Eigen::VectorXd delta_qddot_;
  Eigen::VectorXd delta_rf_;
};

class WBIC : public WBC {
public:
  WBIC(const std::vector<bool> &act_qdot_list,
       const Eigen::MatrixXd *Ji = nullptr);
  ~WBIC() = default;

  // Compute joint commands (jpos, jvel, jacc) using nullspace projection method
  bool FindConfiguration(const Eigen::VectorXd &curr_jpos,
                         const std::map<std::string, Task *> &task_map,
                         const std::map<std::string, Contact *> &contact_map,
                         Eigen::VectorXd &jpos_cmd, Eigen::VectorXd &jvel_cmd,
                         Eigen::VectorXd &wbc_qddot_cmd);

  // Compute joint torque using QP
  bool MakeTorque(const Eigen::VectorXd &wbc_qddot_cmd,
                  const std::map<std::string, ForceTask *> &force_task_map,
                  const std::map<std::string, Contact *> &contact_map,
                  Eigen::VectorXd &jtrq_cmd, void *extra_input = nullptr);

private:
  void _PseudoInverse(const Eigen::MatrixXd &jac, Eigen::MatrixXd &jac_inv);
  void _WeightedPseudoInverse(const Eigen::MatrixXd &jac,
                              const Eigen::MatrixXd &W,
                              Eigen::MatrixXd &jac_bar);
  void _BuildProjectionMatrix(const Eigen::MatrixXd &jac, Eigen::MatrixXd &N,
                              const Eigen::MatrixXd *W = nullptr);

  void
  _BuildContactMtxVect(const std::map<std::string, Contact *> &contact_map);
  void _GetDesiredReactionForce(
      const std::map<std::string, ForceTask *> &force_task_map);
  void _SetQPCost();
  void _SetQPEqualityConstraint(const Eigen::VectorXd &wbc_qddot_cmd);
  void _SetQPInEqualityConstraint();
  void _SolveQP();
  void _GetSolution(const Eigen::VectorXd &wbc_qddot_cmd,
                    Eigen::VectorXd &jtrq_cmd);

  double threshold_;

  // internal constraint nullspace
  Eigen::MatrixXd Ni_dyn_;

  // contact
  Eigen::MatrixXd Jc_;
  Eigen::MatrixXd Uf_mat_;
  Eigen::MatrixXd Uf_vec_;
  int dim_contact_;

  // reaction force
  Eigen::VectorXd des_rf_;

  // QP variables
  WBICData *qp_data_;

  // ProxQP variables
  /*
   min 0.5 * x H x + g.T x
   s.t.
        Ax = b
        l <= Cx <= u
  */
  // cost
  Eigen::MatrixXd H_;
  // Eigen::VectorXd g_;
  //  equality
  Eigen::MatrixXd A_;
  Eigen::VectorXd b_;
  // inequality
  Eigen::MatrixXd C_;
  Eigen::VectorXd l_;
  // Eigen::VectorXd u_; // no upper bound for now
};

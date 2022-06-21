#pragma once

class Task;
class Contact;
class InternalConstraint;
class ForceTask;
class IHWBC {
public:
  IHWBC(const Eigen::MatrixXd &Sa, const Eigen::MatrixXd *Sf = NULL,
        const Eigen::MatrixXd *Sv = NULL);
  virtual ~IHWBC();

  void UpdateSetting(const Eigen::MatrixXd &A, const Eigen::MatrixXd &Ainv,
                     const Eigen::VectorXd &cori, const Eigen::VectorXd &grav);
  void
  Solve(const std::vector<Task *> &task_container,
        const std::vector<Contact *> &contact_container,
        const std::vector<InternalConstraint *> &internal_constraints_container,
        const std::vector<ForceTask *> &force_task_container,
        Eigen::VectorXd &qddot_cmd, Eigen::VectorXd &rf_cmd,
        Eigen::VectorXd &trq_cmd);

protected:
  Eigen::MatrixXd A_;
  Eigen::MatrixXd Ainv_;
  Eigen::VectorXd cori_;
  Eigen::VectorXd grav_;

  int num_qdot_;
  int num_active_;
  int num_passive_;
  int num_float_;

  Eigen::MatrixXd Sa_;
  Eigen::MatrixXd Sv_;
  Eigen::MatrixXd Sf_;

  bool b_float_;
  bool b_passive_;
};

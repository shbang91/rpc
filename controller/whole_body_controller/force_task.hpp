#pragma once
#include "controller/whole_body_controller/basic_contact.hpp"
#include "util/util.hpp"

// for wrench task -> in order of torque, force
class ForceTask {
public:
  ForceTask(Contact *contact)
      : contact_(contact), dim_(contact_->Dim()),
        rf_des_(Eigen::VectorXd::Zero(contact_->Dim())),
        rf_cmd_(Eigen::VectorXd::Zero(contact_->Dim())),
        weight_(Eigen::VectorXd::Zero(contact_->Dim())) {
    util::PrettyConstructor(3, "ForceTask");
  };
  virtual ~ForceTask() = default;

  // setter
  void UpdateDesired(const Eigen::VectorXd &rf_des) { rf_des_ = rf_des; };
  void UpdateDesiredToLocal(const Eigen::VectorXd &rf_des) {
    Eigen::MatrixXd local_R_world(contact_->Dim(), contact_->Dim());
    local_R_world.setZero();
    if (contact_->Dim() == 6) {
      local_R_world.topLeftCorner<3, 3>() = contact_->R().transpose();
      local_R_world.bottomRightCorner<3, 3>() = contact_->R().transpose();
      rf_des_ = local_R_world * rf_des;
    } else if (contact_->Dim() == 3) {
      local_R_world = contact_->R().transpose();
      rf_des_ = local_R_world * rf_des;
    } else {
      assert(false);
    }
  };
  void UpdateCmd(const Eigen::VectorXd &rf_cmd) { rf_cmd_ = rf_cmd; };

  // getter
  Eigen::VectorXd DesiredRf() const { return rf_des_; }
  Eigen::VectorXd CmdRf() const { return rf_cmd_; }
  Eigen::VectorXd Weight() const { return weight_; }
  int Dim() const { return dim_; }
  Contact *contact() { return contact_; }

  void SetParameters(const YAML::Node &node, const bool b_sim) {
    try {
      std::string prefix = b_sim ? "sim" : "exp";
      util::ReadParameter(node, prefix + "_weight", weight_);
    } catch (std::runtime_error &e) {
      std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
                << __FILE__ << "]" << std::endl;
      std::exit(EXIT_FAILURE);
    }
  }

protected:
  Contact *contact_;
  int dim_;
  Eigen::VectorXd rf_des_; // reference reaction force value
  Eigen::VectorXd rf_cmd_; // wbc command
  Eigen::VectorXd weight_;
};

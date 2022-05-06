#include "pnc/fixed_draco_pnc/fixed_draco_controller.hpp"
#include "configuration.hpp"
#include "pnc/fixed_draco_pnc/fixed_draco_interface.hpp"
#include "pnc/robot_system/robot_system.hpp"
#include "util/util.hpp"

FixedDracoController::FixedDracoController(RobotSystem *_robot) {
  robot_ = _robot;

  YAML::Node cfg = YAML::LoadFile(THIS_COM "config/fixed_draco/pnc.yaml");
  control_mode_ = util::ReadParameter<int>(cfg["controller"], "control_mode");
}

FixedDracoController::~FixedDracoController() {}

void FixedDracoController::GetCommand(void *command) {
  if (control_mode_ == control_mode::GravityComp) {
    int r_knee_fe_jp_idx = robot_->GetQdotIdx("r_knee_fe_jp");
    int l_knee_fe_jp_idx = robot_->GetQdotIdx("l_knee_fe_jp");
    int r_knee_fe_jd_idx = robot_->GetQdotIdx("r_knee_fe_jd");
    int l_knee_fe_jd_idx = robot_->GetQdotIdx("l_knee_fe_jd");

    Eigen::MatrixXd sa =
        Eigen::MatrixXd::Zero(robot_->n_qdot_ - 2, robot_->n_qdot_);
    int j = 0;
    for (int i = 0; i < sa.cols(); i++) {
      if (i == r_knee_fe_jp_idx || i == l_knee_fe_jp_idx) {
        continue;
      } else {
        sa(j, i) = 1;
        ++j;
      }
    }

    Eigen::MatrixXd jac_int = Eigen::MatrixXd::Zero(2, robot_->n_qdot_);
    jac_int(0, r_knee_fe_jp_idx) = -1;
    jac_int(0, r_knee_fe_jd_idx) = 1;
    jac_int(1, l_knee_fe_jp_idx) = -1;
    jac_int(1, l_knee_fe_jd_idx) = 1;

    Eigen::MatrixXd jac_int_bar = util::WeightedPseudoInverse(
        jac_int, robot_->GetInertiaMatrix().inverse(), 0.0001);
    Eigen::MatrixXd N_int =
        Eigen::MatrixXd::Identity(robot_->n_qdot_, robot_->n_qdot_) -
        jac_int_bar * jac_int;
    Eigen::MatrixXd sa_N_int_bar = util::WeightedPseudoInverse(
        sa * N_int, robot_->GetInertiaMatrix().inverse(), 0.0001);
    Eigen::VectorXd trq =
        sa_N_int_bar.transpose() * N_int.transpose() * robot_->GetGravity();
    Eigen::VectorXd total_trq = sa.transpose() * trq;

    ((FixedDracoCommand *)command)->joint_positions_cmd_ =
        robot_->EigenVectorToMap(robot_->joint_positions_);
    ((FixedDracoCommand *)command)->joint_velocities_cmd_ =
        robot_->EigenVectorToMap(robot_->joint_velocities_);
    ((FixedDracoCommand *)command)->joint_torques_cmd_ =
        robot_->EigenVectorToMap(total_trq);
  }
}

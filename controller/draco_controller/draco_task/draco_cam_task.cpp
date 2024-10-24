#include "controller/draco_controller/draco_task/draco_cam_task.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"

DracoCAMTask::DracoCAMTask(PinocchioRobotSystem *robot) : Task(robot, 3) {
  util::PrettyConstructor(3, "DracoCAMTask");

  sp_ = DracoStateProvider::GetStateProvider();
}

void DracoCAMTask::UpdateOpCommand(const Eigen::Matrix3d &world_R_local) {
  vel_ = sp_->cam_est_;
  // std::cout << "cam: " << vel_.transpose() << std::endl;

  vel_err_ = des_vel_ - vel_;

  local_des_vel_ = world_R_local.transpose() * des_vel_;
  local_vel_ = world_R_local.transpose() * vel_;
  local_vel_err_ = local_des_vel_ - local_vel_;

  op_cmd_ = des_acc_ + world_R_local * kd_.cwiseProduct(local_vel_err_);
}

void DracoCAMTask::UpdateJacobian() {
  // jacobian_ = robot_->GetAg().topRows<3>();
  Eigen::MatrixXd J_cam =
      robot_->GetIg().inverse() * robot_->GetAg(); // I^-1 * J_cam
  jacobian_ = J_cam.topRows<3>();
}

void DracoCAMTask::UpdateJacobianDotQdot() {
  jacobian_dot_q_dot_ = Eigen::VectorXd::Zero(dim_);
}

void DracoCAMTask::SetParameters(const YAML::Node &node,
                                 const WBC_TYPE wbc_type) {
  try {
    util::ReadParameter(node, "kd", kd_);
    if (wbc_type == WBC_TYPE::IHWBC)
      util::ReadParameter(node, "weight", weight_);
  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  } catch (const std::invalid_argument &ex) {
    std::cerr << "Error: " << ex.what() << " at file: [" << __FILE__ << "]"
              << std::endl;
    std::exit(EXIT_FAILURE);
  }
}

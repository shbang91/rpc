#include "controller/draco_controller/draco_task/draco_com_xy_task.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/filter/digital_filters.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"

#include <cmath>
#include <stdexcept>

DracoCoMXYTask::DracoCoMXYTask(PinocchioRobotSystem *robot)
    : Task(robot, 2), b_sim_(false),
      feedback_source_(feedback_source::kCoMFeedback) {
  util::PrettyConstructor(3, "DracoCoMXYTask");

  sp_ = DracoStateProvider::GetStateProvider();

#if B_USE_MATLOGGER
  logger_ = XBot::MatLogger2::MakeLogger("/tmp/icp_error");
#endif
}

DracoCoMXYTask::~DracoCoMXYTask() {
  if (icp_integrator_ != nullptr)
    delete icp_integrator_;
}

void DracoCoMXYTask::UpdateOpCommand() {
  Eigen::Vector2d com_xy_pos = robot_->GetRobotComPos().head<2>();
  Eigen::Vector2d com_xy_vel = b_sim_ ? robot_->GetRobotComLinVel().head<2>()
                                      : sp_->com_vel_est_.head<2>();

  pos_ << com_xy_pos[0], com_xy_pos[1];
  vel_ << com_xy_vel[0], com_xy_vel[1];

  if (feedback_source_ == feedback_source::kCoMFeedback) {

    pos_err_ = des_pos_ - pos_;
    vel_err_ = des_vel_ - vel_;

    op_cmd_ =
        des_acc_ + kp_.cwiseProduct(pos_err_) + kd_.cwiseProduct(vel_err_);

  } else if (feedback_source_ == feedback_source::kIcpFeedback) {

    double omega = sqrt(kGravAcc / sp_->des_com_height_);

    Eigen::Vector2d des_icp = des_pos_ + des_vel_ / omega;
    Eigen::Vector2d des_icp_dot = des_vel_ + des_acc_ / omega;

    Eigen::Vector2d icp = sp_->dcm_.head<2>();
    Eigen::Vector2d icp_error = des_icp - icp;

    Eigen::Vector2d des_cmp =
        icp - des_icp_dot / omega - kp_.cwiseProduct(icp_error);

    // calculate icp integral error
    Eigen::Vector2d icp_avg_err = Eigen::Vector2d::Zero();
    if (icp_integrator_type_ == icp_integrator::kExponentialSmoother) {
      Eigen::VectorXd error = Eigen::VectorXd::Zero(2);
      error << icp_error[0], icp_error[1];
      icp_integrator_->Input(error);
      Eigen::VectorXd avg_err = icp_integrator_->Output();
      icp_avg_err << avg_err[0], avg_err[1];
    } else if (icp_integrator_type_ == icp_integrator::kLeakyIntegrator) {
      // TODO: clean up this
      icp_avg_err = icp_error * sp_->servo_dt_ + leaky_rate_ * icp_integral_;
      icp_integral_ = util::Clamp2DVector(icp_avg_err, -leaky_integrator_limit_,
                                          leaky_integrator_limit_);
      icp_avg_err = icp_integral_;
    }

    // calculate com x_ddot
    op_cmd_ = omega * omega * (com_xy_pos - des_cmp) +
              omega * omega * ki_.cwiseProduct(icp_avg_err);

#if B_USE_MATLOGGER
    if (sp_->count_ % sp_->data_save_freq_ == 0) {
      logger_->add("icp_error_raw", icp_error);
      logger_->add("icp_avg_err", icp_avg_err);
    }
#endif
  }
}

void DracoCoMXYTask::UpdateJacobian() {
  jacobian_ = robot_->GetComLinJacobian().topRows<2>();
}

void DracoCoMXYTask::UpdateJacobianDotQdot() {
  jacobian_dot_q_dot_ = robot_->GetComLinJacobianDotQdot().head<2>();
}

void DracoCoMXYTask::SetParameters(const YAML::Node &node, const bool b_sim) {
  try {

    b_sim_ = b_sim;

    util::ReadParameter(node, "com_feedback_source", feedback_source_);

    std::string prefix = b_sim ? "sim" : "exp";
    if (feedback_source_ == feedback_source::kCoMFeedback) {
      util::ReadParameter(node, prefix + "_kp", kp_);
      util::ReadParameter(node, prefix + "_kd", kd_);
      util::ReadParameter(node, prefix + "_weight", weight_);
    } else if (feedback_source_ == feedback_source::kIcpFeedback) {
      util::ReadParameter(node, prefix + "_icp_kp", kp_);
      util::ReadParameter(node, prefix + "_icp_kd", kd_);
      util::ReadParameter(node, prefix + "_icp_ki", ki_);
      util::ReadParameter(node, prefix + "_icp_weight", weight_);

      icp_integrator_type_ =
          util::ReadParameter<int>(node, "icp_integrator_type");
      if (icp_integrator_type_ == icp_integrator::kExponentialSmoother) {
        double time_constant =
            util::ReadParameter<double>(node, prefix + "_time_constant");
        Eigen::VectorXd average_icp_error_limit =
            util::ReadParameter<Eigen::VectorXd>(
                node, prefix + "_avg_icp_error_limit");

        icp_integrator_ = new ExponentialMovingAverageFilter(
            sp_->servo_dt_, time_constant, Eigen::VectorXd::Zero(2),
            -average_icp_error_limit, average_icp_error_limit);
      } else if (icp_integrator_type_ == icp_integrator::kLeakyIntegrator) {
        leaky_rate_ = util::ReadParameter<double>(node, prefix + "_leaky_rate");
        leaky_integrator_limit_ = util::ReadParameter<Eigen::Vector2d>(
            node, prefix + "_leaky_integrator_limit");
      }
    } else
      throw std::invalid_argument("No Matching CoM Feedback Source");

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

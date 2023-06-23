#include "controller/draco_controller/draco_task/draco_com_xy_task.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/draco_controller/draco_state_provider.hpp"
#include "controller/filter/digital_filters.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"

#include "util/util.hpp"

#include <cmath>
#include <stdexcept>

#if B_USE_ZMQ
#include "controller/draco_controller/draco_data_manager.hpp"
#endif

DracoCoMXYTask::DracoCoMXYTask(PinocchioRobotSystem *robot)
    : Task(robot, 2), b_sim_(false),
      feedback_source_(feedback_source::kCoMFeedback),
      icp_integrator_(nullptr),
      icp_lpf_(nullptr) {
  util::PrettyConstructor(3, "DracoCoMXYTask");

  sp_ = DracoStateProvider::GetStateProvider();

#if B_USE_MATLOGGER
  logger_ = XBot::MatLogger2::MakeLogger("/tmp/draco_icp_data");
#endif
}

DracoCoMXYTask::~DracoCoMXYTask() {
  if (icp_integrator_ != nullptr)
    delete icp_integrator_;
  if (icp_lpf_ != nullptr)
    delete icp_lpf_;
}

void DracoCoMXYTask::UpdateOpCommand(const Eigen::Matrix3d &rot_world_local) {
  Eigen::Vector2d com_xy_pos = robot_->GetRobotComPos().head<2>();
  Eigen::Vector2d com_xy_vel = b_sim_ ? robot_->GetRobotComLinVel().head<2>()
                                      : sp_->com_vel_est_.head<2>();

  pos_ << com_xy_pos[0], com_xy_pos[1];
  vel_ << com_xy_vel[0], com_xy_vel[1];

  pos_err_ = des_pos_ - pos_;
  vel_err_ = des_vel_ - vel_;

  Eigen::Matrix2d rot_link_w =
      rot_world_local.transpose().topLeftCorner<2, 2>();
  // std::cout <<
  // "============================================================="
  //<< std::endl;
  // util::PrettyPrint(rot_link_w, std::cout, "com xy original local rot mat");
  // rot_link_w.col(0).normalize();
  // rot_link_w.col(1).normalize();
  // util::PrettyPrint(rot_link_w, std::cout, "com xy normalized local rot
  // mat"); std::cout << "cross product result: "
  //<< rot_link_w.col(0).dot(rot_link_w.col(1)) << std::endl;

  //=============================================================
  // local com xy task data
  //=============================================================
  // TODO: notice that torso Rx, Ry need to be precisely controlled
  local_des_pos_ = rot_link_w * des_pos_;
  local_pos_ = rot_link_w * pos_;
  local_pos_err_ = rot_link_w * pos_err_;

  local_des_vel_ = rot_link_w * des_vel_;
  local_vel_ = rot_link_w * vel_;
  local_vel_err_ = rot_link_w * vel_err_;

  local_des_acc_ = rot_link_w * des_acc_;

  if (feedback_source_ == feedback_source::kCoMFeedback) {
    //=============================================================
    // operational space command
    //=============================================================
    // op_cmd_ =
    // des_acc_ + kp_.cwiseProduct(pos_err_) + kd_.cwiseProduct(vel_err_);
    op_cmd_ =
        des_acc_ + rot_link_w.transpose() * (kp_.cwiseProduct(local_pos_err_) +
                                             kd_.cwiseProduct(local_vel_err_));

  } else if (feedback_source_ == feedback_source::kIcpFeedback) {

    double com_height = robot_->GetRobotComPos()[2];
    double omega = sqrt(kGravAcc / com_height);

    Eigen::Vector2d des_icp = des_pos_ + des_vel_ / omega;
    Eigen::Vector2d des_icp_dot = des_vel_ + des_acc_ / omega;

    Eigen::Vector2d local_des_icp = rot_link_w * des_icp;
    Eigen::Vector2d local_des_icp_dot = rot_link_w * des_icp_dot;

    Eigen::Vector2d icp = sp_->dcm_.head<2>();
    Eigen::Vector2d icp_err = des_icp - icp;

    Eigen::Vector2d local_icp = rot_link_w * icp;
    Eigen::Vector2d local_icp_err = rot_link_w * icp_err;

    // Eigen::Vector2d des_cmp =
    // icp - des_icp_dot / omega - kp_.cwiseProduct(icp_err);

    //=============================================================
    // calculate icp integral error
    //=============================================================
    Eigen::Vector2d icp_avg_err = Eigen::Vector2d::Zero();
    if (icp_integrator_type_ == icp_integrator::kExponentialSmoother) {
      Eigen::VectorXd error = Eigen::VectorXd::Zero(2);
      error << icp_err[0], icp_err[1];
      icp_integrator_->Input(error);
      Eigen::VectorXd avg_err = icp_integrator_->Output();
      icp_avg_err << avg_err[0], avg_err[1];
    } else if (icp_integrator_type_ == icp_integrator::kLeakyIntegrator) {
      // TODO: clean up this

      // filter local ICP error used with kp gain
      icp_integrator_->Input(local_icp_err);
      local_icp_err = icp_integrator_->Output();
      // icp_lpf_->Input(local_icp_err);
      // local_icp_err = icp_lpf_->Output();

      // apply leaky integration
      icp_avg_err = icp_err * sp_->servo_dt_ + leaky_rate_ * icp_integral_;
      icp_integral_ = util::Clamp2DVector(icp_avg_err, -leaky_integrator_limit_,
                                          leaky_integrator_limit_);
      icp_avg_err = icp_integral_;
    }

    Eigen::Vector2d local_icp_avg_err = rot_link_w * icp_avg_err;

    //=============================================================
    // calculate operational space command
    //=============================================================
    Eigen::Vector2d des_cmp =
            icp - des_icp_dot / omega -
            rot_link_w.transpose() * (kp_.cwiseProduct(local_icp_err)) -
            rot_link_w.transpose() * (ki_.cwiseProduct(local_icp_avg_err));

    // op_cmd_ = omega * omega * (com_xy_pos - des_cmp) +
    // omega * omega * ki_.cwiseProduct(icp_avg_err);

//    op_cmd_ = omega * omega * (com_xy_pos - des_cmp) +
//              omega * omega * rot_link_w.transpose() *
//                  (ki_.cwiseProduct(local_icp_avg_err));
    op_cmd_ = omega * omega * (com_xy_pos - des_cmp);

#if B_USE_ZMQ
    if (sp_->count_ % sp_->data_save_freq_ == 0) {
      DracoDataManager *dm = DracoDataManager::GetDataManager();
      dm->data_->des_icp = des_icp;
      dm->data_->des_cmp = des_cmp;
    }
#endif

#if B_USE_MATLOGGER
    if (sp_->count_ % sp_->data_save_freq_ == 0) {
      logger_->add("des_icp", des_icp);
      // logger_->add("act_icp", icp); same as the "est_icp" in state estimator
      logger_->add("local_des_icp", local_des_icp);
      logger_->add("local_act_icp", local_icp);
      logger_->add("icp_error_raw", icp_err);
      logger_->add("icp_avg_err", icp_avg_err); // used for feedback control
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

      double time_constant =
              util::ReadParameter<double>(node, prefix + "_time_constant");
      Eigen::VectorXd average_icp_error_limit =
              util::ReadParameter<Eigen::VectorXd>(
                      node, prefix + "_avg_icp_error_limit");
      icp_integrator_ = new ExponentialMovingAverageFilter(
              sp_->servo_dt_, time_constant, Eigen::VectorXd::Zero(2),
              -average_icp_error_limit, average_icp_error_limit);

      icp_integrator_type_ =
          util::ReadParameter<int>(node, "icp_integrator_type");
      if (icp_integrator_type_ == icp_integrator::kExponentialSmoother) {

      } else if (icp_integrator_type_ == icp_integrator::kLeakyIntegrator) {
        leaky_rate_ = util::ReadParameter<double>(node, prefix + "_leaky_rate");
        leaky_integrator_limit_ = util::ReadParameter<Eigen::Vector2d>(
            node, prefix + "_leaky_integrator_limit");

        double lpf_time_constant = util::ReadParameter<double>(
            node, prefix + "_lpf_time_constant");
        icp_lpf_ = new FirstOrderLowPassFilter(sp_->servo_dt_,
            lpf_time_constant, 2);
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

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
    : Task(robot, 2), feedback_source_(feedback_source::kCoMFeedback),
      icp_integrator_(nullptr), icp_lpf_(nullptr) {
  util::PrettyConstructor(3, "DracoCoMXYTask");

  sp_ = DracoStateProvider::GetStateProvider();

#if B_USE_MATLOGGER
  // logger_ = XBot::MatLogger2::MakeLogger("/tmp/draco_icp_data");
#endif
}

DracoCoMXYTask::~DracoCoMXYTask() {
  if (icp_integrator_ != nullptr)
    delete icp_integrator_;
  if (icp_lpf_ != nullptr)
    delete icp_lpf_;
}

void DracoCoMXYTask::UpdateOpCommand(const Eigen::Matrix3d &world_R_local) {
  Eigen::Vector2d com_xy_pos = robot_->GetRobotComPos().head<2>();
  Eigen::Vector2d com_xy_vel = sp_->com_vel_est_.head<2>();

  pos_ << com_xy_pos[0], com_xy_pos[1];
  vel_ << com_xy_vel[0], com_xy_vel[1];

  pos_err_ = des_pos_ - pos_;
  vel_err_ = des_vel_ - vel_;

  Eigen::Matrix2d local_R_world =
      world_R_local.transpose().topLeftCorner<2, 2>();
  // std::cout <<
  // "============================================================="
  //<< std::endl;
  // util::PrettyPrint(local_R_world, std::cout, "com xy original local rot
  // mat"); local_R_world.col(0).normalize(); local_R_world.col(1).normalize();
  // util::PrettyPrint(local_R_world, std::cout, "com xy normalized local rot
  // mat"); std::cout << "cross product result: "
  //<< local_R_world.col(0).dot(local_R_world.col(1)) << std::endl;

  //=============================================================
  // local com xy task data
  //=============================================================
  // TODO: notice that torso Rx, Ry need to be precisely controlled
  local_des_pos_ = local_R_world * des_pos_;
  local_pos_ = local_R_world * pos_;
  local_pos_err_ = local_R_world * pos_err_;

  local_des_vel_ = local_R_world * des_vel_;
  local_vel_ = local_R_world * vel_;
  local_vel_err_ = local_R_world * vel_err_;

  local_des_acc_ = local_R_world * des_acc_;

  if (feedback_source_ == feedback_source::kCoMFeedback) {
    //=============================================================
    // operational space command
    //=============================================================
    op_cmd_ = des_acc_ +
              local_R_world.transpose() * (kp_.cwiseProduct(local_pos_err_) +
                                           kd_.cwiseProduct(local_vel_err_));

  } else if (feedback_source_ == feedback_source::kIcpFeedback) {

    double com_height = robot_->GetRobotComPos()[2];
    double omega = sqrt(kGravAcc / com_height);
    // double omega = sqrt(kGravAcc / sp_->des_com_height_);

    Eigen::Vector2d des_icp = des_pos_ + des_vel_ / omega;
    Eigen::Vector2d des_icp_dot = des_vel_ + des_acc_ / omega;

    Eigen::Vector2d local_des_icp = local_R_world * des_icp;
    Eigen::Vector2d local_des_icp_dot = local_R_world * des_icp_dot;

    Eigen::Vector2d icp = sp_->dcm_.head<2>();
    Eigen::Vector2d icp_err = des_icp - icp;

    Eigen::Vector2d local_icp = local_R_world * icp;
    Eigen::Vector2d local_icp_err = local_R_world * icp_err;

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

      // uncomment to filter ICP error before numerical integral
      //       icp_lpf_->Input(icp_err);
      //       icp_err = icp_lpf_->Output();

      // apply leaky integration
      icp_avg_err = icp_err * sp_->servo_dt_ + leaky_rate_ * icp_integral_;
      icp_integral_ = util::Clamp2DVector(icp_avg_err, -leaky_integrator_limit_,
                                          leaky_integrator_limit_);
      icp_avg_err = icp_integral_;
    }

    Eigen::Vector2d local_icp_avg_err = local_R_world * icp_avg_err;

    //=============================================================
    // calculate operational space command
    //=============================================================
    Eigen::Vector2d des_cmp =
        icp - des_icp_dot / omega -
        local_R_world.transpose() * (kp_.cwiseProduct(local_icp_err)) -
        local_R_world.transpose() * (ki_.cwiseProduct(local_icp_avg_err));

    op_cmd_ = omega * omega * (com_xy_pos - des_cmp);

#if B_USE_ZMQ
    if (sp_->count_ % sp_->data_save_freq_ == 0) {
      DracoDataManager *dm = DracoDataManager::GetDataManager();
      dm->data_->des_icp = des_icp;
      dm->data_->des_cmp = des_cmp;
    }
#endif

#if B_USE_MATLOGGER
    // if (sp_->count_ % sp_->data_save_freq_ == 0) {
    // logger_->add("des_icp", des_icp);
    // logger_->add("act_icp", icp); same as the "est_icp" in state estimator
    // logger_->add("local_des_icp", local_des_icp);
    // logger_->add("local_act_icp", local_icp);
    // logger_->add("icp_error_raw", icp_err);
    // logger_->add("icp_avg_err", icp_avg_err); // used for feedback control
    //}
#endif
  }
}
void DracoCoMXYTask::UpdateJacobian() {
  jacobian_ = robot_->GetComLinJacobian().topRows<2>();
}

void DracoCoMXYTask::UpdateJacobianDotQdot() {
  jacobian_dot_q_dot_ = robot_->GetComLinJacobianDotQdot().head<2>();
}

void DracoCoMXYTask::SetParameters(const YAML::Node &node,
                                   const WBC_TYPE wbc_type) {
  try {
    util::ReadParameter(node, "com_feedback_source", feedback_source_);

    if (wbc_type == WBC_TYPE::IHWBC) {
      if (feedback_source_ == feedback_source::kCoMFeedback)
        util::ReadParameter(node, "weight", weight_);
      else if (feedback_source_ == feedback_source::kIcpFeedback)
        util::ReadParameter(node, "icp_weight", weight_);
    } else if (wbc_type == WBC_TYPE::WBIC)
      util::ReadParameter(node, "kp_ik", kp_ik_);

    if (feedback_source_ == feedback_source::kCoMFeedback) {
      util::ReadParameter(node, "kp", kp_);
      util::ReadParameter(node, "kd", kd_);
    } else if (feedback_source_ == feedback_source::kIcpFeedback) {
      util::ReadParameter(node, "icp_kp", kp_);
      util::ReadParameter(node, "icp_kd", kd_);
      util::ReadParameter(node, "icp_ki", ki_);

      icp_integrator_type_ =
          util::ReadParameter<int>(node, "icp_integrator_type");
      if (icp_integrator_type_ == icp_integrator::kExponentialSmoother) {
        double time_constant =
            util::ReadParameter<double>(node, "time_constant");
        Eigen::VectorXd average_icp_error_limit =
            util::ReadParameter<Eigen::VectorXd>(node, "avg_icp_error_limit");

        icp_integrator_ = new ExponentialMovingAverageFilter(
            sp_->servo_dt_, time_constant, Eigen::VectorXd::Zero(2),
            -average_icp_error_limit, average_icp_error_limit);
      } else if (icp_integrator_type_ == icp_integrator::kLeakyIntegrator) {
        leaky_rate_ = util::ReadParameter<double>(node, "leaky_rate");
        leaky_integrator_limit_ = util::ReadParameter<Eigen::Vector2d>(
            node, "leaky_integrator_limit");
        double lpf_time_constant =
            util::ReadParameter<double>(node, "lpf_time_constant");
        // TODO icp_lpf_ can be completely removed
        icp_lpf_ =
            new FirstOrderLowPassFilter(sp_->servo_dt_, lpf_time_constant, 2);
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

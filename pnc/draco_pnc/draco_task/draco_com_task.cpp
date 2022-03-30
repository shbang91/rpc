#include "pnc/draco_pnc/draco_task/draco_com_task.hpp"

DracoComTask::DracoComTask(RobotSystem *_robot, const int &_feedback_source, 
        const int &_com_height_target) : Task(_robot, 3){

    feedback_source_ = _feedback_source;
    com_height_target_ = _com_height_target;

    sp_ = DracoStateProvider::GetStateProvider();
}

DracoComTask::~DracoComTask(){}

void DracoComTask::UpdateOscCommand(){
    if (feedback_source_ == feedback_source::kComFeedback){
        Eigen::Vector3d com_pos = robot_->GetRobotComPos();
        Eigen::Vector3d com_vel = robot_->GetRobotComLinVel();

        pos_ << com_pos[0], com_pos[1], com_pos[2];
        vel_ << com_vel[0], com_vel[1], com_vel[2];

        if (com_height_target_ == com_height_target::kComHeight){
            pos_[2] = robot_->GetRobotComPos()[2];
            vel_[2] = robot_->GetRobotComLinVel()[2];
        } else if (com_height_target_ == com_height_target::kBaseHeight){
            pos_[2] = robot_->GetLinkIso("torso_com_link").translation()[2];
            vel_[2] = robot_->GetLinkVel("torso_com_link")[5];
        } else {
            throw std::invalid_argument("No Matching CoM Height Task Target");
        }

        pos_err_ = des_pos_ - pos_;
        vel_err_ = des_vel_ - vel_;

        osc_cmd_ = des_acc_ + kp_.cwiseProduct(pos_err_) + kd_.cwiseProduct(vel_err_);

    } else if (feedback_source_ == feedback_source::kDcmFeedback){
        Eigen::Vector3d com_pos = robot_->GetRobotComPos();
        Eigen::Vector3d com_vel = robot_->GetRobotComLinVel();

        double omega = sqrt(9.81/des_pos_[2]);
        Eigen::Vector2d des_dcm = des_pos_.head(2) + des_vel_.head(2) / omega;
        Eigen::Vector2d des_dcm_dot = des_vel_.head(2) + des_acc_.head(2) / omega;

        if (com_height_target_ == com_height_target::kComHeight){
            pos_[2] = robot_->GetRobotComPos()[2];
            vel_[2] = robot_->GetRobotComLinVel()[2];
        } else if (com_height_target_ == com_height_target::kBaseHeight){
            pos_[2] = robot_->GetLinkIso("torso_com_link").translation()[2];
            vel_[2] = robot_->GetLinkVel("torso_com_link")[5];
        } else {
            throw std::invalid_argument("No Matching CoM Height Task Target");
        }

        Eigen::Vector2d des_cmp = sp_->dcm_.head(2) + omega * des_dcm_dot 
            + kp_.head(2).cwiseProduct(sp_->dcm_.head(2) - des_dcm);

        osc_cmd_.head(2) = omega * omega * (com_pos.head(2) - des_cmp);
        osc_cmd_[2] = kp_[2] * (des_pos_[2] - pos_[2]) + kd_[2] * (des_vel_[2] - vel_[2]);
    } else {
        throw std::invalid_argument("No Matching Feedback Source on CoM Task");
    }
}

void DracoComTask::UpdateTaskJacobian(){
    if (com_height_target_ == com_height_target::kComHeight){
        jacobian_ = robot_->GetComLinJacobian();
    } else if (com_height_target_ == com_height_target::kBaseHeight){
        jacobian_.block(0,0,2,robot_->n_qdot_) = robot_->GetComLinJacobian().block(0,0,2,robot_->n_qdot_);
        jacobian_.block(2,0,1,robot_->n_qdot_) = robot_->GetLinkJacobian("torso_com_link").block(5,0,1,robot_->n_qdot_);
    } else {
        throw std::invalid_argument("No Matching CoM Task Jacobian");
    }
}

void DracoComTask::UpdateTaskJacobianDotQdot(){
    if (com_height_target_ == com_height_target::kComHeight){
        jacobian_dot_q_dot_ = robot_->GetComLinJacobianDot() * robot_->GetQdot();
    } else if (com_height_target_ == com_height_target::kBaseHeight){
        jacobian_dot_q_dot_.head(2) = (robot_->GetComLinJacobianDot() * robot_->GetQdot()).head(2);
        jacobian_dot_q_dot_[2] = robot_->GetLinkJacobianDotQdot("torso_com_link")[5]; 
    } else {
        throw std::invalid_argument("No Matching CoM Task JacobianDotQdot");
    }
}

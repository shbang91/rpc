#include "pnc/draco_pnc/draco_state_estimator.hpp"

DracoStateEstimator::DracoStateEstimator(RobotSystem *_robot){
    robot_ = _robot;
    sp_ = DracoStateProvider::GetStateProvider();

    base_joint_position_.setZero();
    base_joint_orientation_.setIdentity();
    base_com_position_.setZero();

    prev_base_joint_position_.setZero();
    prev_base_com_position_.setZero();

    base_joint_lin_vel_.setZero();
    base_com_lin_vel_.setZero();

    global_leg_odometry_.setZero();

}

DracoStateEstimator::~DracoStateEstimator(){};

void DracoStateEstimator::InitializeModel(DracoSensorData *_sensor_data){
    this->UpdateModel(_sensor_data);

}

void DracoStateEstimator::UpdateModel(DracoSensorData *_sensor_data){

    //Estimate floating base orientation
    this->EstimateOrientation(_sensor_data);

    //Update robot model only with base orientation
    robot_->UpdateRobotModel(Eigen::Vector3d::Zero(),
        Eigen::Quaternion<double>(base_joint_orientation_),
        Eigen::Vector3d::Zero(),
        _sensor_data->imu_frame_velocities_.head(3),
        Eigen::Vector3d::Zero(),
        Eigen::Quaternion<double>(base_joint_orientation_),
        Eigen::Vector3d::Zero(),
        _sensor_data->imu_frame_velocities_.head(3),
        _sensor_data->joint_positions_,
        _sensor_data->joint_velocities_,
        false);

    //Estimate floating base position
    this->EstimatePosition(_sensor_data);

    //Update robot model with full estimate
    robot_->UpdateRobotModel(base_com_position_,
        Eigen::Quaternion<double>(base_joint_orientation_),
        base_com_lin_vel_,
        _sensor_data->imu_frame_velocities_.head(3),
        base_joint_position_,
        Eigen::Quaternion<double>(base_joint_orientation_),
        base_joint_lin_vel_,
        _sensor_data->imu_frame_velocities_.head(3),
        _sensor_data->joint_positions_,
        _sensor_data->joint_velocities_,
        true);

    //foot contact switch
    if (_sensor_data->b_lf_contact_) {
       sp_->b_lf_contact_ = true;
    } else {
       sp_->b_lf_contact_ = false;
    }

    if (_sensor_data->b_rf_contact_) {
       sp_->b_rf_contact_ = true;
    } else {
       sp_->b_rf_contact_ = false;
    }

    //TODO:velocity filtering for com vel (for real experiment)

    //compute dcm
    this->ComputeDCM();


    //Debugging purpose
    //std::cout << "===================" << std::endl;
    //std::cout << "======PnC Robot====" << std::endl;
    //std::cout << "===================" << std::endl;
    //std::cout << "base com pos" << std::endl;
    //std::cout << base_com_position_ << std::endl;
    //std::cout << "base com ori" << std::endl;
    //std::cout << robot_->GetLinkIso("torso_com_link").linear() << std::endl;
    //std::cout << "base joint pos" << std::endl;
    //std::cout << base_joint_position_ << std::endl;
    //std::cout << "base joint ori" << std::endl;
    //std::cout << base_joint_orientation_ << std::endl;
    //std::cout << "base_com_lin_vel" << std::endl;
    //std::cout << base_com_lin_vel_ << std::endl;
    //std::cout << "base_com_ang_vel" << std::endl;
    //std::cout << robot_->GetLinkVel("torso_com_link").head(3) << std::endl;
    //std::cout << "base_joint_lin_vel" << std::endl;
    //std::cout << base_joint_lin_vel_ << std::endl;
    //std::cout << "base_joint_ang_vel" << std::endl;
    //std::cout << robot_->GetLinkVel("torso_com_link").head(3) << std::endl;
    //std::cout << "imu_frame_ang_vel" << std::endl;
    //std::cout << robot_->GetLinkVel("torso_imu").head(3) << std::endl;
    //std::cout << "imu_frame_lin_vel" << std::endl;
    //std::cout << robot_->GetLinkVel("torso_imu").tail(3) << std::endl;
    //std::cout << "lf pos" << std::endl;
    //std::cout << robot_->GetLinkIso("l_foot_contact").translation() << std::endl;
}

void DracoStateEstimator::EstimateOrientation(DracoSensorData* _sensor_data){
    Eigen::Matrix3d R_0_imu = robot_->GetLinkIso("torso_imu").linear();
    Eigen::Matrix3d R_0_base_com = robot_->GetLinkIso("torso_com_link").linear(); 
    Eigen::Matrix3d R_imu_base_com = R_0_imu.transpose() * R_0_base_com;

    //estimate orientation
    base_joint_orientation_ = _sensor_data->imu_frame_isometry_.block(0,0,3,3) * R_imu_base_com;
}

void DracoStateEstimator::EstimatePosition(DracoSensorData* _sensor_data){
    //anchor frame depending on the stance foot
    Eigen::Vector3d anchor_frame_pos = robot_->GetLinkIso(sp_->stance_foot_).translation();
    Eigen::Vector3d anchor_frame_vel = robot_->GetLinkVel(sp_->stance_foot_).tail(3);

    if (sp_->stance_foot_ != sp_->prev_stance_foot_) {
        Eigen::Vector3d anchor_frame_pos_diff = anchor_frame_pos - robot_->GetLinkIso(sp_->prev_stance_foot_).translation();
        global_leg_odometry_ += anchor_frame_pos_diff;
    }

    //estimate position
    base_joint_position_ = global_leg_odometry_ - anchor_frame_pos;
    base_com_position_ = base_joint_position_ + base_joint_orientation_ * robot_->GetBaseLocalComPos();

    static bool first_visit(true);
    if (first_visit) {
       prev_base_joint_position_ = base_joint_position_; 
       prev_base_com_position_ = base_com_position_; 
       first_visit = false;
    }

    //estimate base linear velocity
    base_joint_lin_vel_ =
        (base_joint_position_ - prev_base_joint_position_) / sp_->servo_dt_;
    base_com_lin_vel_ =
        (base_com_position_ - prev_base_com_position_) / sp_->servo_dt_;

    //save current time step data
    sp_->prev_stance_foot_ = sp_->stance_foot_;
    prev_base_joint_position_ = base_joint_position_;
    prev_base_com_position_ = base_com_position_;

}

void DracoStateEstimator::ComputeDCM(){
    Eigen::Vector3d com_pos = robot_->GetRobotComPos();
    Eigen::Vector3d com_vel = robot_->GetRobotComLinVel();
    double omega = sqrt(9.81/com_pos[2]);

    sp_->prev_dcm_ = sp_->dcm_;
    sp_->dcm_ = com_pos + com_vel / omega;

    double cutoff_period = 0.01; //10ms cut-off period for first-order low pass filter
    double alpha = sp_->servo_dt_ / cutoff_period;

    sp_->dcm_vel_ = alpha * (sp_->dcm_ - sp_->prev_dcm_) / sp_->servo_dt_ 
                    + (1 - alpha) *sp_->dcm_vel_;

}

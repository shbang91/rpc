#pragma once
#include <iostream>
#include <Eigen/Dense>

#include "pnc/robot_system/robot_system.hpp"


class Task {
    public:
        Task(RobotSystem *_robot, const int &_dim){
            robot_ = _robot;
            dim_ = _dim;

            des_pos_ = Eigen::VectorXd::Zero(dim_);
            des_vel_ = Eigen::VectorXd::Zero(dim_);
            des_acc_ = Eigen::VectorXd::Zero(dim_);

            pos_ = Eigen::VectorXd::Zero(dim_);
            vel_ = Eigen::VectorXd::Zero(dim_);

            pos_err_ = Eigen::VectorXd::Zero(dim_);
            vel_err_ = Eigen::VectorXd::Zero(dim_);

            kp_ = Eigen::VectorXd::Zero(dim_);
            kd_ = Eigen::VectorXd::Zero(dim_);
            ki_ = Eigen::VectorXd::Zero(dim_);

            osc_cmd_ = Eigen::VectorXd::Zero(dim_);

            jacobian_ = Eigen::MatrixXd::Zero(dim_,robot_->n_qdot_);
            jacobian_dot_q_dot_ = Eigen::VectorXd::Zero(dim_);

            task_component_hierarchy_ = Eigen::VectorXd::Zero(dim_);
        }
        virtual ~Task(){}

        // for orientation task, _des_pos is a 4 dimensional vector [w,x,y,z]
        // for angular momentum task, _des_pos is ignored
        void UpdateDesiredTask(const Eigen::VectorXd &_des_pos, const Eigen::VectorXd &_des_vel, const Eigen::VectorXd &_des_acc){
            des_pos_ = _des_pos;
            des_vel_ = _des_vel;
            des_acc_ = _des_acc;
        }

        virtual void UpdateOscCommand() = 0;

        virtual void UpdateTaskJacobian() = 0;
        virtual void UpdateTaskJacobianDotQdot() = 0;



    protected:
        RobotSystem *robot_;
        int dim_;

        //desired quantities
        Eigen::VectorXd des_pos_;
        Eigen::VectorXd des_vel_;
        Eigen::VectorXd des_acc_;

        //measured quantities
        Eigen::VectorXd pos_;
        Eigen::VectorXd vel_;

        Eigen::VectorXd pos_err_;
        Eigen::VectorXd vel_err_;

        Eigen::VectorXd kp_;
        Eigen::VectorXd kd_;
        Eigen::VectorXd ki_;

        Eigen::VectorXd osc_cmd_;

        Eigen::VectorXd jacobian_;
        Eigen::VectorXd jacobian_dot_q_dot_;

        Eigen::VectorXd task_component_hierarchy_;

};

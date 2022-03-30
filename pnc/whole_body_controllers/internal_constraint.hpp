#pragma once
#include <Eigen/Dense>

#include "pnc/robot_system/robot_system.hpp"

class InternalConstraint {
    public:
        InternalConstraint(RobotSystem *_robot, const int &_dim){
            robot_ = _robot;
            dim_ = _dim;

            jacobian_ = Eigen::MatrixXd::Zero(dim_, robot_->n_qdot_);
            jacobian_dot_q_dot_ = Eigen::VectorXd::Zero(dim_);
        };
        virtual ~InternalConstraint();

        virtual void UpdateInternalConstraintJacobian() = 0;
        virtual void UpdateInternalConstraintJacobianDotQdot() = 0;

        Eigen::MatrixXd jacobian_;
        Eigen::VectorXd jacobian_dot_q_dot_;

    protected:
        RobotSystem *robot_;
        int dim_;

}

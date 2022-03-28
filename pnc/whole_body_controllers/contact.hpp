#pragma once
#include <Eigen/Dense>
#include <string>

#include "pnc/robot_system/robot_system.hpp"

class Contact {
    public:
        Contact(RobotSystem *_robot, const int &_dim, 
                const std::string &_target_link, const double &_mu){

            robot_ = _robot;
            dim_ = _dim;
            target_link_ = _target_link;
            mu_ = _mu;

            jacobian_ = Eigen::MatrixXd::Zero(dim_, robot_->n_qdot_);
            jacobian_dot_q_dot_ = Eigen::VectorXd::Zero(dim_);

            rf_z_max_ = 1000.;
        };
        virtual ~Contact(){};

        Eigen::MatrixXd jacobian_;
        Eigen::VectorXd jacobian_dot_q_dot_;

        double rf_z_max_;
        Eigen::MatrixXd cone_constraint_matrix_;
        Eigen::VectorXd cone_constraint_vector_;

        virtual void UpdateContactJacobian() = 0;
        virtual void UpdateContactJacobianDotQdot() = 0;
        virtual void UpdateConeConstraint() = 0;

    protected:
        RobotSystem *robot_;

        int dim_;
        std::string target_link_;
        double mu_;
};

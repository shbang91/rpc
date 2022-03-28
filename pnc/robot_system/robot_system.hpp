#pragma once
#include <Eigen/Dense>
#include <string>
#include <map>

class RobotSystem {
    public:
        RobotSystem(const bool &_b_fixed_base, const bool &_b_print_info):
            b_fixed_base_(_b_fixed_base), b_print_info_(_b_print_info) {};

        virtual ~RobotSystem() {};

        int n_floating_base_;
        int n_q_;
        int n_qdot_;
        int n_a_;

        double total_mass_;

        Eigen::Matrix<double, Eigen::Dynamic, 2> joint_pos_limits_;
        Eigen::Matrix<double, Eigen::Dynamic, 2> joint_vel_limits_;
        Eigen::Matrix<double, Eigen::Dynamic, 2> joint_trq_limits_;

        Eigen::VectorXd joint_positions_;
        Eigen::VectorXd joint_velocities_;

        //Centroidal inertial tensor
        Eigen::Matrix<double, 6, 6> Ig_;

        //Centroidal momentum matrix(mapping qdot to centroidal momentum quantities)
        Eigen::Matrix<double, 6, Eigen::Dynamic> Ag_;

        //Centroidal momentum
        Eigen::Matrix<double, 6, 1> Hg_;

        virtual int GetQIdx(const std::string &_joint_name) = 0;
        virtual int GetQdotIdx(const std::string &_joint_name) = 0;
        virtual int GetJointIdx(const std::string &_joint_name) = 0;

        virtual std::map<std::string, double> EigenVectorToMap(const Eigen::VectorXd &_joint_cmd) = 0;
        virtual Eigen::VectorXd MapToEigenVector(std::map<std::string, double> _joint_map) = 0;

        virtual Eigen::Vector3d GetBaseLocalComPos() = 0;
        virtual std::string GetBaseLinkName() = 0;

        //Configure the following properties: n_floating, n_q, n_q_dot, n_a,
        //total_mass, joint_pos_limit, joint_vel_limit, joint_trq_limit.
        virtual void ConfigRobot() = 0;

        virtual void UpdateRobotModel(const Eigen::Vector3d &_base_com_pos,
                                      const Eigen::Quaternion<double> &_base_com_quat,
                                      const Eigen::Vector3d &_base_com_lin_vel,
                                      const Eigen::Vector3d &_base_com_ang_vel,
                                      const Eigen::Vector3d &_base_joint_pos,
                                      const Eigen::Quaternion<double> &_base_joint_quat,
                                      const Eigen::Vector3d &_base_joint_lin_vel,
                                      const Eigen::Vector3d &_base_joint_ang_vel,
                                      std::map<std::string, double> _joint_positions,
                                      std::map<std::string, double> _joint_velocities,
                                      const bool _b_update_centroid = false) = 0;

        //Update centroid quantities (Ig_, Ag_, Hg_)
        virtual void UpdateCentroidalQuantities() = 0;

        virtual Eigen::VectorXd GetQ() = 0;
        virtual Eigen::VectorXd GetQdot() = 0;
        virtual Eigen::VectorXd GetInertiaMatrix() = 0;
        virtual Eigen::VectorXd GetGravity() = 0;
        virtual Eigen::VectorXd GetCoriolis() = 0;
        virtual Eigen::Vector3d GetRobotComPos() = 0;
        virtual Eigen::Vector3d GetRobotComLinVel() = 0;

        virtual Eigen::Isometry3d GetLinkIso(const std::string &_link_name) = 0;
        virtual Eigen::Matrix<double, 6, 1> GetLinkVel(const std::string &_link_name) = 0; 
        virtual Eigen::Matrix<double, 6, Eigen::Dynamic> GetLinkJacobian(const std::string &_link_name) = 0; 
        virtual Eigen::Matrix<double, 6, 1> GetLinkJacobianDotQdot(const std::string &_link_name) = 0; 

        virtual Eigen::Matrix<double, 3, Eigen::Dynamic> GetComLinJacobian() = 0;
        virtual Eigen::Matrix<double, 3, Eigen::Dynamic> GetComLinJacobianDot() = 0;

    protected:
        bool b_fixed_base_;
        bool b_print_info_;

};

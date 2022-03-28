#pragma once
#include "pnc/whole_body_controllers/contact.hpp"

class PointContact : public Contact {
    public:
        PointContact(RobotSystem *_robot, const std::string &_target_link,
                const double &_mu);
        virtual ~PointContact();

        void UpdateContactJacobian();
        void UpdateContactJacobianDotQdot();
        void UpdateConeConstraint();

};

class SurfaceContact : public Contact {
    public:
        SurfaceContact(RobotSystem *_robot, const std::string &_target_link,
                const double &_mu, const double &_x, const double &_y);
        virtual ~SurfaceContact();

        void UpdateContactJacobian();
        void UpdateContactJacobianDotQdot();
        void UpdateConeConstraint();
    private:
        double x_;
        double y_;
};

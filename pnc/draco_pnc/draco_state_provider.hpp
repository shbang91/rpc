#pragma once
#include <iostream>
#include <Eigen/Dense>

class DracoStateProvider{
    public:
        static DracoStateProvider *GetStateProvider();
        ~DracoStateProvider(){};

        double servo_dt_;
        double current_time_;

        //stance foot
        std::string stance_foot_;
        std::string prev_stance_foot_;

        //dcm
        Eigen::Vector3d dcm_;
        Eigen::Vector3d prev_dcm_;
        Eigen::Vector3d dcm_vel_;

        //foot contact
        bool b_lf_contact_;
        bool b_rf_contact_;


    private:
        DracoStateProvider();
};

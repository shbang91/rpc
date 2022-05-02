#pragma once
#include <iostream>
#include <Eigen/Dense>

class FixedDracoStateProvider{
    public:
        static FixedDracoStateProvider *GetStateProvider();
        ~FixedDracoStateProvider(){};

        double servo_dt_;
        double current_time_;

    private:
        FixedDracoStateProvider();
};

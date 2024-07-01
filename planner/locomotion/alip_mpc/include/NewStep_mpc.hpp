

#ifndef NewStep_mpc_HPP
#define NewStep_mpc_HPP
#include <iostream>

#pragma once

#include <string.h>
#include <vector>
#include "util/util.hpp"

#include <casadi/casadi.hpp>

#include "data_structs.hpp"

using namespace std;



class NewStep_mpc
{
public:
    // Constructor
    NewStep_mpc();
    NewStep_mpc(string & horizon, string & intervals, const bool &new_solver);

    // Destructor
    virtual ~NewStep_mpc();

    void SetParameters(const YAML::Node &node);

    // Update
    void Update_(const input_data_t &input_data,
                 output_data_t &output_data, full_horizon_sol &full_sol);
    void setCOMinit(vector<double> com_init){x_com_init_step_ = com_init;}

private:
    bool new_solver_;


    // Related to class loop
    int iter_ = 0;
    // custom_cassie_out struct // for input
    vector<double> x_lip_current_;
    double stance_leg_;
    double zH_;
    double Ts_;
    double Tr_;
    double leg_width_;
    double Lx_offset_;
    double Ly_des_;
    // std::vector<double> ufp_max_;
    // std::vector<double> ufp_min_;
    double kx_new_;
    double ky_new_;
    double mu_new_;

    // Terrain
    double kx_init_;
    deque<double> kx_traj_;
    double ky_init_;
    deque<double> ky_traj_;
    double mu_init_;
    deque<double> mux_traj_;
    deque<double> muy_traj_;



    double mass_;
    double ufp_x_max;  
    double ufp_y_max_;
    double ufp_y_min_;
    double y_mech_max_;
    int leg_identity_ = -1;             // leg swap parameter
    double new_ufp_x_max_;
    double new_ufp_y_max_;


    // Casadi fp solver info
    const int n_xlip_ = 4;
    const int n_ufp_ = 2;

    int N_steps_;
    int N_xsol_;
    int N_ufpsol_;

    string solver_LS_;        // = "qrqptestY";
    string solver_RS_;        // = "qrqptestY";

    casadi::Function f_solver_LS_; // = casadi::external(solver);
    casadi::Function f_solver_RS_; // = casadi::external(solver);
    
    vector<double> xlip_guess_;
    vector<double> ufp_guess_;

    vector<double> x_com_init_step_;

    double q_term_0_;
    double q_term_1_;
    double q_term_2_;
    double q_term_3_;

};

#endif
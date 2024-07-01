#ifndef TEST_MPC_DYN_HPP
#define TEST_MPC_DYN_HPP

#include <vector>
#include "planner/locomotion/alip_mpc/include/data_structs.hpp"
struct test_data_t {
  //input
  //x_lip_current
  double x_c;
  double y_c;
  double L_xc;
  double L_yc;

  double stance_leg;
  double zH;
  double Ts;
  double Tr;
  double leg_width;
  double Lx_offset;
  double Ly_des;
  double kx;
  double ky;
  double mu;
  //output
  double ufp_wrt_st_x;
  double ufp_wrt_st_y;
  double ufp_wrt_com_x;
  double ufp_wrt_com_y;

  //std::vector<double> fullmpc_xsol;
  //std::vector<double> fullmpc_usol;
} ;



#endif



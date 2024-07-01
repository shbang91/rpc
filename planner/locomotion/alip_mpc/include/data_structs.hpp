
#ifndef DATA_STRUCTS_T_HPP
#define DATA_STRUCTS_T_HPP

#include <iostream>
#include<vector>


struct input_data_t{
  double xlip_current[4];
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
} ;


struct output_data_t{
  double ufp_wrt_st[2];
  double ufp_wrt_com[2];
} ;  //foot placement


struct full_horizon_sol{
  std::vector<double> xlip_sol;
  std::vector<double> ufp_sol;
} ;  //foot placement


#endif
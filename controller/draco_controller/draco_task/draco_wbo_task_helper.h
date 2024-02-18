/* This file was automatically generated by CasADi.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

extern "C" int Q_xyz_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int Q_xyz_func_alloc_mem(void);
extern "C" int Q_xyz_func_init_mem(int mem);
extern "C" void Q_xyz_func_free_mem(int mem);
extern "C" int Q_xyz_func_checkout(void);
extern "C" void Q_xyz_func_release(int mem);
extern "C" void Q_xyz_func_incref(void);
extern "C" void Q_xyz_func_decref(void);
extern "C" casadi_int Q_xyz_func_n_in(void);
extern "C" casadi_int Q_xyz_func_n_out(void);
extern "C" casadi_real Q_xyz_func_default_in(casadi_int i);
extern "C" const char* Q_xyz_func_name_in(casadi_int i);
extern "C" const char* Q_xyz_func_name_out(casadi_int i);
extern "C" const casadi_int* Q_xyz_func_sparsity_in(casadi_int i);
extern "C" const casadi_int* Q_xyz_func_sparsity_out(casadi_int i);
extern "C" int Q_xyz_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define Q_xyz_func_SZ_ARG 190
#define Q_xyz_func_SZ_RES 2
#define Q_xyz_func_SZ_IW 0
#define Q_xyz_func_SZ_W 969
extern "C" int jac_Q_xyz_func(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem);
extern "C" int jac_Q_xyz_func_alloc_mem(void);
extern "C" int jac_Q_xyz_func_init_mem(int mem);
extern "C" void jac_Q_xyz_func_free_mem(int mem);
extern "C" int jac_Q_xyz_func_checkout(void);
extern "C" void jac_Q_xyz_func_release(int mem);
extern "C" void jac_Q_xyz_func_incref(void);
extern "C" void jac_Q_xyz_func_decref(void);
extern "C" casadi_int jac_Q_xyz_func_n_in(void);
extern "C" casadi_int jac_Q_xyz_func_n_out(void);
extern "C" casadi_real jac_Q_xyz_func_default_in(casadi_int i);
extern "C" const char* jac_Q_xyz_func_name_in(casadi_int i);
extern "C" const char* jac_Q_xyz_func_name_out(casadi_int i);
extern "C" const casadi_int* jac_Q_xyz_func_sparsity_in(casadi_int i);
extern "C" const casadi_int* jac_Q_xyz_func_sparsity_out(casadi_int i);
extern "C" int jac_Q_xyz_func_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w);
#define jac_Q_xyz_func_SZ_ARG 5
#define jac_Q_xyz_func_SZ_RES 190
#define jac_Q_xyz_func_SZ_IW 0
#define jac_Q_xyz_func_SZ_W 2169

#pragma once

#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/force_task.hpp"
#include "planner/locomotion/alip_mpc/include/NewStep_mpc.hpp"
#include "util/interpolation.hpp"
#include <Eigen/Dense>

class Task;
class ForceTask;
class PinocchioRobotSystem;
class NewStep_mpc;

class AlipMpcTrajectoryManager {
public:
  AlipMpcTrajectoryManager(NewStep_mpc *alipMpc, Task *com_xy_task,
                           Task *com_z_task, Task *torso_ori, Task *lfoot_task,
                           Task *lfoot_ori, Task *rfoot_task, Task *rfoot_ori,
                           ForceTask *lf_force_task, ForceTask *rg_force_task,
                           PinocchioRobotSystem *robot);

  virtual ~AlipMpcTrajectoryManager();

  void initializeOri();
  void setNewOri(const double &des_com_yaw);
  void MpcSolutions(const double &tr_, const double &st_leg_,
                    const double &Lx_offset_, const double &Ly_des_,
                    const double &com_yaw_, const double &kx_,
                    const double &ky_, const double &mu_, const bool &first);

  Eigen::Vector3d add_residual_rl_action(const Eigen::VectorXd &action);
  void safety_proj();

  void InertiaToMpcCoordinates(const bool &first);
  void OutputMpcToInertiaCoordinates();

  void turning_self_collision();
  void GenerateTrajs(const double &tr_, const bool &ori_traj);

  void saveDoubleStanceFoot();
  void UpdateDoubleStance();
  void UpdateCurrentOri(Task *task);
  void UpdateCurrentPos(Task *task);

  void UpdateDesired(const double t);
  void MakeParallelToGround(Eigen::Isometry3d &pose);
  double ComputeZpos(const double &x, const double &y, const double &zH_);

  void SetSwingFootStart(Eigen::Vector3d pos) { Swingfoot_start = pos; }
  void SetLydes(double des) { indata.Ly_des = des; }

  // for testing
  void saveTrajectories(const double start_time, const double dt,
                        const double end_time);
  void saveCurrentCOMstate(const double t);
  void saveMpcCOMstate(const double t);
  void saveSwingState(const double t);
  void saveRobotCommand(const double t);
  void saveCOMstateWorld(const double t);
  void saveCOMstateMPCcoor(const double t);
  // Getters
  input_data_t GetIndata() { return indata; }
  output_data_t GetOutdata() { return outdata; }
  full_horizon_sol GetFullsol() { return fullsol; }

  void SetParameters(const YAML::Node &node);
  void SetTaskWeights(const YAML::Node &node);

  Eigen::Isometry3d Get_des_end_torso_iso() { return des_end_torso_iso_; }
  int printCounter;

private:
  NewStep_mpc *alipMpc;
  Task *com_xy_task;
  Task *com_z_task;
  Task *torso_ori;
  Task *lfoot_task;
  Task *lfoot_ori;
  Task *rfoot_task;
  Task *rfoot_ori;
  ForceTask *lf_force_task;
  ForceTask *rg_force_task;
  PinocchioRobotSystem *robot_;

  AlipSwing2 *AlipSwingPos2;

  HermiteQuaternionCurve *torso_ori_curve_;
  HermiteQuaternionCurve *swfoot_ori_curve_;

  full_horizon_sol fullsol; // refrence frame wrt current stance leg
  input_data_t indata;      // reference frame wrt current stance leg
  output_data_t outdata;
  double com_yaw_;

  Eigen::Vector3d Swingfoot_start;

  Eigen::Vector3d Swingfoot_end;
  Eigen::Vector3d Swingfootvel_end = Eigen::Vector3d::Zero();

  Eigen::Vector3d stleg_pos;
  Eigen::Vector3d stleg_pos_torso_ori;

  Eigen::Quaterniond des_end_swfoot_quat_;
  Eigen::Isometry3d des_end_swfoot_iso_;
  Eigen::Quaterniond des_end_torso_quat_;
  Eigen::Isometry3d des_end_torso_iso_;

  bool first_ever;

  double mass;
  double swing_height; // this will be reference for step of 0.5 m will use
                       // linear swing heigh with (0,0)
  double reference_swing_height; // will have to change this, also can do a
                                 // variable swing height
  int variable_height;           // can have problems with small steps

  std::fstream file1;
  std::fstream file2;
  std::fstream file3;
  std::fstream file4;
  std::fstream file5;
  std::fstream file6;
  std::fstream file7;
  std::fstream file8;
  std::fstream file9;
  std::fstream file10;
  std::fstream file11;

  int saveCounter;
  double refzH;
  Eigen::Vector3d terrain; // normalised (-kx, -ky, 1)

  // task weights
  Eigen::VectorXd com_z_task_weight;
  Eigen::Vector2d com_xy_task_weight;
  Eigen::Vector3d torso_ori_weight;
  Eigen::Vector3d swing_foot_weight;
  Eigen::Vector3d stance_foot_weight;
  Eigen::Vector3d stance_foot_ori_weight;
  Eigen::Vector3d swing_foot_ori_weight;
  // swing foot task
  Eigen::VectorXd des_swfoot_pos;
  Eigen::VectorXd des_swfoot_vel;
  Eigen::VectorXd des_swfoot_acc;

  Eigen::Quaterniond des_swfoot_quat_;
  Eigen::Vector3d des_swfoot_ang_vel_;
  Eigen::Vector3d des_swfoot_ang_acc_;

  Eigen::Quaterniond des_torso_quat_;
  Eigen::Vector3d des_torso_ang_vel_;
  Eigen::Vector3d des_torso_ang_acc_;

  Eigen::Quaterniond start_torso_quat_;
  Eigen::Quaterniond start_swfoot_quat_;

  double indataLz;
  double ufp_x_max_;
  double ufp_y_min_;
  double ufp_y_max_;

  Eigen::Vector3d stance_rfoot;
  Eigen::Vector3d stance_lfoot;
  Eigen::Quaterniond stance_rfoot_quat;
  Eigen::Quaterniond stance_lfoot_quat;
  Eigen::VectorXd stance_rfoot_quat_coef;
  Eigen::VectorXd stance_lfoot_quat_coef;
};
/* TODO:
- change the way we update the task weights not necessary to do at every loop

*/

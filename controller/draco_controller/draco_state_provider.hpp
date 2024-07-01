#pragma once
#include <Eigen/Dense>
#include <vector>
#include "util/util.hpp"

class DracoStateProvider {
public:
  static DracoStateProvider *GetStateProvider();
  ~DracoStateProvider() = default;

  // servo dt should be set outside of controller
  double servo_dt_;
  int data_save_freq_;

  int count_;
  double current_time_;

  // should be set outside of controller
  Eigen::VectorXd nominal_jpos_;

  // used in pos estimate in estimator module
  int stance_foot_;
  int prev_stance_foot_;

  Eigen::Matrix3d rot_world_local_;

  Eigen::Vector3d dcm_;
  Eigen::Vector3d prev_dcm_;
  Eigen::Vector3d dcm_vel_;

  bool b_lf_contact_;
  bool b_rf_contact_;
  bool b_request_change_swing_leg_;
  int b_swing_leg_;

  Eigen::Vector3d com_vel_est_;

  int state_;
  int prev_state_;

  bool b_use_base_height_;
  bool b_use_kf_state_estimator_;

  double des_com_height_;
  Eigen::Quaterniond des_torso_quat_;

  int planning_id_;

  std::vector<int> floating_base_jidx_;

  Eigen::Vector3d cam_est_;

  //RL action
  Eigen::VectorXd res_rl_action_;

  //mpc 
  int mpc_freq_;
  //WBC obs
  double mass_;
  int initial_stance_leg_;
  bool rl_trigger_ = false;
  double stance_leg_;
  double Lx_offset_des_;
  double Ly_des_;
  double des_com_yaw_;
  double Ts_;
  double Tr_;
  Eigen::Isometry3d des_end_torso_iso_;   //at the end of the swing
  Eigen::Vector3d com_pos_stance_frame_;  //rotatin frame is at the end of the swing
  Eigen::Vector3d L_stance_frame_;
  Eigen::Vector3d stfoot_pos_;
  Eigen::Vector3d torso_roll_pitch_yaw_;   //at the time of the computation
  Eigen::Vector3d full_policy_;
  Eigen::Vector3d swfoot_roll_pitch_yaw_;  //at the time of the computation
  Eigen::Vector3d torso_com_ang_vel_;
  Eigen::Vector3d torso_rpy_des_frame;
  Eigen::Vector3d swfoot_rpy_des_frame; 


  Eigen::VectorXd get_wbc_obs(){
    Eigen::VectorXd obs(33);

    obs <<  stance_leg_,
            Lx_offset_des_,           //2
            Ly_des_,
            des_com_yaw_,             //4
            com_pos_stance_frame_(0), //5
            com_pos_stance_frame_(1),
            com_pos_stance_frame_(2),
            L_stance_frame_(0),       //8
            L_stance_frame_(1),
            L_stance_frame_(2),
            stfoot_pos_(0),           //11
            stfoot_pos_(1),
            stfoot_pos_(2),
            torso_roll_pitch_yaw_(0), //14
            torso_roll_pitch_yaw_(1),
            torso_roll_pitch_yaw_(2), //16
            swfoot_roll_pitch_yaw_(0),//17
            swfoot_roll_pitch_yaw_(1),
            swfoot_roll_pitch_yaw_(2),//19
            Tr_,                      //20              //Ts_,
            full_policy_(0),          //21
            full_policy_(1),
            full_policy_(2),          //23
            state_,                   //24
            torso_com_ang_vel_(0),
            torso_com_ang_vel_(1),
            torso_com_ang_vel_(2),    //27   
            torso_rpy_des_frame(0), 
            torso_rpy_des_frame(1), 
            torso_rpy_des_frame(2),   //30
            swfoot_rpy_des_frame(0), 
            swfoot_rpy_des_frame(1), 
            swfoot_rpy_des_frame(2);  //33

    return obs;

  }

  double mu_;
  double kx_;
  double ky_;

  void outsideCommand(const YAML::Node &node){
    //util::ReadParameter(node, "Lx_offset", Lx_offset_);
    //util::ReadParameter(node, "Ly_des", Ly_des_);
    //util::ReadParameter(node, "com_yaw", des_com_yaw_);
    util::ReadParameter(node, "mu", mu_);
    util::ReadParameter(node, "kx", kx_);
    util::ReadParameter(node, "ky", ky_);
    //des_com_yaw_ *= M_PI/180;

}

void Reset();


private:
  DracoStateProvider();
};

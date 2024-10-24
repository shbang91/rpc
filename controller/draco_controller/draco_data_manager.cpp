#include <controller/draco_controller/draco_data_manager.hpp>

DracoDataManager *DracoDataManager::GetDataManager() {
  static DracoDataManager dm;
  return &dm;
}

DracoDataManager::DracoDataManager() {
  // draco data initialize
  data_ = std::make_unique<DracoData>();

  // zmq stuff initialize
  context_ = std::make_unique<zmq::context_t>(1);
  socket_ = std::make_unique<zmq::socket_t>(*context_, ZMQ_PUB);

  // boolean initialize
  b_initialize_socket_ = false;
}

bool DracoDataManager::IsInitialized() { return b_initialize_socket_; }

void DracoDataManager::InitializeSocket(const std::string &ip_address) {
  socket_->bind(ip_address);
  b_initialize_socket_ = true;
}

void DracoDataManager::SendData() {
  assert(b_initialize_socket_);

  draco::pnc_msg msg;
  msg.set_time(data_->time_);
  msg.set_phase(data_->phase_);

  for (int i(0); i < 3; ++i) {
    msg.add_est_base_joint_pos(data_->est_base_joint_pos_[i]);
    msg.add_est_base_joint_ori(data_->est_base_joint_ori_[i]);
    msg.add_kf_base_joint_pos(data_->kf_base_joint_pos_[i]);
    msg.add_kf_base_joint_ori(data_->kf_base_joint_ori_[i]);
    msg.add_des_com_pos(data_->des_com_pos_[i]);
    msg.add_act_com_pos(data_->act_com_pos_[i]);
    msg.add_lfoot_pos(data_->lfoot_pos_[i]);
    msg.add_rfoot_pos(data_->rfoot_pos_[i]);
    msg.add_lfoot_ori(data_->lfoot_ori_[i]);
    msg.add_rfoot_ori(data_->rfoot_ori_[i]);
  }
  msg.add_est_base_joint_ori(data_->est_base_joint_ori_[3]);
  msg.add_kf_base_joint_ori(data_->kf_base_joint_ori_[3]);
  msg.add_lfoot_ori(data_->lfoot_ori_[3]);
  msg.add_rfoot_ori(data_->rfoot_ori_[3]);

  for (int i(0); i < data_->joint_positions_.size(); i++)
    msg.add_joint_positions(data_->joint_positions_[i]);

  for (int i(0); i < data_->lfoot_rf_cmd_.size(); i++)
    msg.add_lfoot_rf_cmd(data_->lfoot_rf_cmd_[i]);

  for (int i(0); i < data_->rfoot_rf_cmd_.size(); i++)
    msg.add_rfoot_rf_cmd(data_->rfoot_rf_cmd_[i]);

  // contact sensing measurements
  msg.set_b_lfoot(data_->b_lfoot_);
  msg.set_b_rfoot(data_->b_rfoot_);
  msg.set_lfoot_volt_normal_raw(data_->lfoot_volt_normal_raw_);
  msg.set_rfoot_volt_normal_raw(data_->rfoot_volt_normal_raw_);
  msg.set_lfoot_rf_normal(data_->lfoot_rf_normal_);
  msg.set_rfoot_rf_normal(data_->rfoot_rf_normal_);
  msg.set_lfoot_rf_normal_filt(data_->lfoot_rf_normal_filt_);
  msg.set_rfoot_rf_normal_filt(data_->rfoot_rf_normal_filt_);

  for (int i(0); i < 2; i++) {
    msg.add_est_icp(data_->est_icp[i]);
    msg.add_des_icp(data_->des_icp[i]);
    msg.add_des_cmp(data_->des_cmp[i]);
    msg.add_com_xy_weight(data_->com_xy_weight[i]);
    msg.add_com_xy_kp(data_->com_xy_kp[i]);
    msg.add_com_xy_kd(data_->com_xy_kd[i]);
    msg.add_com_xy_ki(data_->com_xy_ki[i]);
  }

  msg.set_com_z_weight(data_->com_z_weight);
  msg.set_com_z_kp(data_->com_z_kp);
  msg.set_com_z_kd(data_->com_z_kd);

  for (int i(0); i < 3; i++) {
    msg.add_torso_ori_weight(data_->torso_ori_weight[i]);
    msg.add_torso_ori_kp(data_->torso_ori_kp[i]);
    msg.add_torso_ori_kd(data_->torso_ori_kd[i]);

    msg.add_lf_pos_weight(data_->lf_pos_weight[i]);
    msg.add_lf_pos_kp(data_->lf_pos_kp[i]);
    msg.add_lf_pos_kd(data_->lf_pos_kd[i]);

    msg.add_rf_pos_weight(data_->rf_pos_weight[i]);
    msg.add_rf_pos_kp(data_->rf_pos_kp[i]);
    msg.add_rf_pos_kd(data_->rf_pos_kd[i]);

    msg.add_lf_ori_weight(data_->lf_ori_weight[i]);
    msg.add_lf_ori_kp(data_->lf_ori_kp[i]);
    msg.add_lf_ori_kd(data_->lf_ori_kd[i]);

    msg.add_rf_ori_weight(data_->rf_ori_weight[i]);
    msg.add_rf_ori_kp(data_->rf_ori_kp[i]);
    msg.add_rf_ori_kd(data_->rf_ori_kd[i]);
  }

  for (int i(0); i < 4; i++) {
    // quaternion in order (w,x,y,z)
    msg.add_quat_world_local(data_->quat_world_local_.coeffs()[i]);
  }

  // =============================================================
  // MPC variables
  // =============================================================
  draco::Pos com_msg;
  for (const auto &des_com : data_->des_com_traj) {
    com_msg.set_x(des_com(0));
    com_msg.set_y(des_com(1));
    com_msg.set_z(des_com(2));
    msg.add_des_com_traj()->CopyFrom(com_msg);
  }

  draco::Euler ori_msg;
  for (const auto &des_ori : data_->des_torso_ori_traj) {
    ori_msg.set_x(des_ori(0));
    ori_msg.set_y(des_ori(1));
    ori_msg.set_z(des_ori(2));
    msg.add_des_torso_ori_traj()->CopyFrom(ori_msg);
  }

  draco::Pos lf_pos_msg;
  for (const auto &des_pos : data_->des_lf_pos_traj) {
    lf_pos_msg.set_x(des_pos(0));
    lf_pos_msg.set_y(des_pos(1));
    lf_pos_msg.set_z(des_pos(2));
    msg.add_des_lf_pos_traj()->CopyFrom(lf_pos_msg);
  }
  draco::Pos rf_pos_msg;
  for (const auto &des_pos : data_->des_rf_pos_traj) {
    rf_pos_msg.set_x(des_pos(0));
    rf_pos_msg.set_y(des_pos(1));
    rf_pos_msg.set_z(des_pos(2));
    msg.add_des_rf_pos_traj()->CopyFrom(rf_pos_msg);
  }

  draco::Euler lf_ori_msg;
  for (const auto &des_ori : data_->des_lf_ori_traj) {
    lf_ori_msg.set_x(des_ori(0));
    lf_ori_msg.set_y(des_ori(1));
    lf_ori_msg.set_z(des_ori(2));
    msg.add_des_lf_ori_traj()->CopyFrom(lf_ori_msg);
  }

  draco::Euler rf_ori_msg;
  for (const auto &des_ori : data_->des_rf_ori_traj) {
    rf_ori_msg.set_x(des_ori(0));
    rf_ori_msg.set_y(des_ori(1));
    rf_ori_msg.set_z(des_ori(2));
    msg.add_des_rf_ori_traj()->CopyFrom(rf_ori_msg);
  }
  // serialize msg in string type
  std::string encoded_msg;
  msg.SerializeToString(&encoded_msg);

  // send data
  zmq::message_t zmq_msg(encoded_msg.size());
  memcpy((void *)zmq_msg.data(), encoded_msg.c_str(), encoded_msg.size());
  socket_->send(zmq_msg);
}

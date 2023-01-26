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
    msg.add_des_com_pos(data_->des_com_pos_[i]);
    msg.add_act_com_pos(data_->act_com_pos_[i]);
    msg.add_lfoot_pos(data_->lfoot_pos_[i]);
    msg.add_rfoot_pos(data_->rfoot_pos_[i]);
    msg.add_lfoot_ori(data_->lfoot_ori_[i]);
    msg.add_rfoot_ori(data_->rfoot_ori_[i]);
  }
  msg.add_est_base_joint_ori(data_->est_base_joint_ori_[3]);
  msg.add_lfoot_ori(data_->lfoot_ori_[3]);
  msg.add_rfoot_ori(data_->rfoot_ori_[3]);

  for (int i(0); i < data_->joint_positions_.size(); i++)
    msg.add_joint_positions(data_->joint_positions_[i]);

  for (int i(0); i < data_->lfoot_rf_cmd_.size(); i++)
    msg.add_lfoot_rf_cmd(data_->lfoot_rf_cmd_[i]);

  for (int i(0); i < data_->rfoot_rf_cmd_.size(); i++)
    msg.add_rfoot_rf_cmd(data_->rfoot_rf_cmd_[i]);

  for (int i(0); i < 2; i++) {
    msg.add_est_icp(data_->est_icp[i]);
    msg.add_des_icp(data_->des_icp[i]);
    msg.add_des_cmp(data_->des_cmp[i]);
  }

  // for (int i = 0; i < 3; ++i) {
  // msg.add_base_joint_pos(data_->base_joint_pos_[i]);
  // msg.add_base_joint_ori(data_->base_joint_ori_[i]);
  // msg.add_base_joint_lin_vel(data_->base_joint_lin_vel_[i]);
  // msg.add_base_joint_ang_vel(data_->base_joint_ang_vel_[i]);
  //}
  // msg.add_base_joint_ori(data_->base_joint_ori_[3]);

  // TODO:TEST
  // draco::msg_list msg_list;

  // draco::fb_msg *fb_msg = msg_list.add_fb();
  // draco::fb_msg::base_joint_pos *bjoint_pos = fb_msg->add_bjoint_pos();
  // for (int i(0); i < 3; ++i)
  // bjoint_pos->add_xyz(data_->base_joint_pos_[i]);
  // TODO:TEST

  // serialize msg in string type
  std::string encoded_msg;
  msg.SerializeToString(&encoded_msg);

  // send data
  zmq::message_t zmq_msg(encoded_msg.size());
  memcpy((void *)zmq_msg.data(), encoded_msg.c_str(), encoded_msg.size());
  socket_->send(zmq_msg);
}

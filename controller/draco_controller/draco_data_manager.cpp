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

void DracoDataManager::InitializeSocket(const std::string &ip_address) {
  socket_->bind(ip_address);
  b_initialize_socket_ = true;
}

void DracoDataManager::SendData() {
  assert(b_initialize_socket_);

  draco::pnc_msg msg;
  msg.set_time(data_->time_);

  for (int i = 0; i < 3; ++i) {
    msg.add_base_joint_pos(data_->base_joint_pos_[i]);
    msg.add_base_joint_ori(data_->base_joint_ori_[i]);
    msg.add_base_joint_lin_vel(data_->base_joint_lin_vel_[i]);
    msg.add_base_joint_ang_vel(data_->base_joint_ang_vel_[i]);
  }
  msg.add_base_joint_ori(data_->base_joint_ori_[3]);

  for (int i = 0; i < data_->joint_positions_.size(); ++i) {
    msg.add_joint_positions(data_->joint_positions_[i]);
  }
  for (int i = 0; i < 3; ++i) {
    msg.add_est_base_joint_pos(data_->est_base_joint_pos_[i]);
    msg.add_est_base_joint_ori(data_->est_base_joint_ori_[i]);
    msg.add_est_base_joint_lin_vel(data_->est_base_joint_lin_vel_[i]);
    msg.add_est_base_joint_ang_vel(data_->est_base_joint_ang_vel_[i]);
  }
  msg.add_est_base_joint_ori(data_->est_base_joint_ori_[3]);

  for (int i(0); i < 3; ++i) {
    msg.add_des_com_pos(data_->des_com_pos_[i]);
    msg.add_act_com_pos(data_->act_com_pos_[i]);
    msg.add_des_com_vel(data_->des_com_vel_[i]);
    msg.add_act_com_vel(data_->act_com_vel_[i]);
  }

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

DracoData::DracoData()
    : time_(0.), base_joint_pos_(Eigen::Vector3d ::Zero()),
      base_joint_ori_(Eigen::Vector4d::Zero()),
      base_joint_lin_vel_(Eigen::Vector3d::Zero()),
      base_joint_ang_vel_(Eigen::Vector3d::Zero()),
      est_base_joint_pos_(Eigen::Vector3d::Zero()),
      est_base_joint_ori_(Eigen::Vector4d::Zero()),
      est_base_joint_lin_vel_(Eigen::Vector3d::Zero()),
      est_base_joint_ang_vel_(Eigen::Vector3d::Zero()),
      joint_positions_(Eigen::VectorXd::Zero(27)),
      torques_(Eigen::VectorXd::Zero(27)),
      des_com_pos_(Eigen::VectorXd::Zero(3)),
      act_com_pos_(Eigen::VectorXd::Zero(3)),
      des_com_vel_(Eigen::VectorXd::Zero(3)),
      act_com_vel_(Eigen::VectorXd::Zero(3)) {}

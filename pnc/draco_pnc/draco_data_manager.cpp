#include <pnc/draco_pnc/draco_data_manager.hpp>

DracoDataManager *DracoDataManager::GetDataManager(){
    static DracoDataManager dm;
    return &dm;
}

DracoDataManager::DracoDataManager(){
    //dracodata initialize
    data_ = std::make_unique<DracoData>();

    //zmq stuff initialize
    context_ = std::make_unique<zmq::context_t>(1);
    socket_ = std::make_unique<zmq::socket_t>(*context_, ZMQ_PUB);

    //boolean initialize
    b_initialize_socket_ = false;
}

void DracoDataManager::InitializeSocket(const std::string &ip_address){
    socket_->bind(ip_address);
    b_initialize_socket_ = true;
}

DracoDataManager::~DracoDataManager(){}

void DracoDataManager::SendData(){
    assert(b_initialize_socket_);

    draco::pnc_msg msg;
    msg.set_time(data_->time_);

    for (int i = 0; i < 3; ++i) {
       msg.add_base_joint_pos(data_->base_joint_pos_[i]);
       msg.add_base_joint_ori(data_->base_joint_ori_[i]);
       msg.add_base_joint_lin_vel(data_->base_joint_lin_vel_[i]);
       msg.add_base_joint_ang_vel(data_->base_joint_ang_vel_[i]);

       msg.add_base_com_pos(data_->base_com_pos_[i]);
       msg.add_base_com_ori(data_->base_com_ori_[i]);
       msg.add_base_com_lin_vel(data_->base_com_lin_vel_[i]);
       msg.add_base_com_ang_vel(data_->base_com_ang_vel_[i]);
    }
    msg.add_base_joint_ori(data_->base_joint_ori_[3]);
    msg.add_base_com_ori(data_->base_com_ori_[3]);

    for (int i = 0; i < data_->joint_positions_.size(); ++i) {
        msg.add_joint_positions(data_->joint_positions_[i]);
    }


    //serialize msg in string type
    std::string encoded_msg;
    msg.SerializeToString(&encoded_msg);

    //send data
    zmq::message_t zmq_msg(encoded_msg.size());
    memcpy ((void *) zmq_msg.data(), encoded_msg.c_str(),
            encoded_msg.size());
    socket_->send(zmq_msg);

}

DracoData::DracoData(){
    time_ = 0.;

    base_joint_pos_ = Eigen::Vector3d::Zero();
    base_joint_ori_ = Eigen::Vector4d::Zero();
    base_joint_lin_vel_ = Eigen::Vector3d::Zero();
    base_joint_ang_vel_ = Eigen::Vector3d::Zero();

    base_com_pos_ = Eigen::Vector3d::Zero();
    base_com_ori_ = Eigen::Vector4d::Zero();
    base_com_lin_vel_ = Eigen::Vector3d::Zero();
    base_com_ang_vel_ = Eigen::Vector3d::Zero();

    joint_positions_ = Eigen::VectorXd::Zero(27);


}

DracoData::~DracoData(){}

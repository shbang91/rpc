#include "controller/model_predictive_controller/nmpc/nmpc_handler.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/whole_body_controller/tci_container.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/force_task.hpp"

#include <tuple>

using namespace google::protobuf::io;

NMPCHandler::NMPCHandler(PinocchioRobotSystem *robot, TCIContainer* tci_container, int lfoot_id, int rfoot_id):
MPCHandler(robot),
tci_container_(tci_container),
lfoot_id_(lfoot_id),
rfoot_id_(rfoot_id),
first_visit_(true)
{
    util::PrettyConstructor(2, "NMPCHandler");

    _resetStepIndex();

    // At the moment, the MPC handles a walk that always starts with the left foot
    robot_side_first_ = end_effector::LFoot;
    solution_received_ = false;

    T_ = 1.4;
    n_nodes_ = 20;
    ctrl_dt_ = 0.00125;
    is_new_ = false;

    std::cout <<  int(T_ / ctrl_dt_) << "      " << int(T_ / n_nodes_ / ctrl_dt_) << std::endl;

    // Initialize vectors for linear interpolation
    old_lf_force_.setZero(6);   old_rf_force_.setZero(6);

    old_com_pos_.setZero(3);    old_com_vel_.setZero(3);    old_com_acc_.setZero(3);
    old_base_ori_.setZero(4);   old_base_vel_.setZero(3);   old_base_acc_.setZero(3);
    old_lf_pos_.setZero(3);     old_lf_vel_.setZero(3);     old_lf_acc_.setZero(3);
    old_rf_pos_.setZero(3);     old_rf_vel_.setZero(3);     old_rf_acc_.setZero(3);

    interp_count_ = 0;

    logger_ = XBot:: MatLogger2::MakeLogger("/tmp/tasks_references");
}

void NMPCHandler::paramInitialization(const YAML::Node &node)
{
  // void setCoMHeight(double z_vrp_in); // Sets the desired CoM Height
  // Load Custom Params ----------------------------------
  try {
    // Load DCM Parameters
    util::ReadParameter(node, "com_height", nominal_com_height_);
    util::ReadParameter(node, "t_double_support", t_contact_transition_);
    util::ReadParameter(node, "t_single_support", t_swing_);

    // Load Walking Primitives Parameters
    util::ReadParameter(node, "nominal_footwidth", nominal_footwidth_);
    util::ReadParameter(node, "nominal_forward_step", nominal_forward_step_);
    util::ReadParameter(node, "nominal_backward_step", nominal_backward_step_);
    util::ReadParameter(node, "nominal_turn_radians", nominal_turn_radians_);
    util::ReadParameter(node, "nominal_strafe_distance", nominal_strafe_distance_);
    util::ReadParameter(node, "n_steps", n_steps_);

  } catch (std::runtime_error &e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
}

void NMPCHandler::walkInPlace()
{
    _resetIndexAndClearFootsteps();
    _populateStepInPlace();
//    _alternateLeg();
}

void NMPCHandler::walkForward()
{
    _resetIndexAndClearFootsteps();
    _populateStepForward();
//    alternateLeg();
}

void NMPCHandler::_populateStepInPlace()
{
  _updateStartingStance(); // Update the starting foot locations of the robot

  FootStep left_footstep = left_foot_stance_;
  FootStep right_footstep = right_foot_stance_;
  FootStep mid_footstep = mid_foot_stance_;

  footstep_list = FootStep::GetInPlaceWalkFootStep(n_steps_, nominal_footwidth_, robot_side_first_, mid_footstep);

  // Add the starting stances
  if (robot_side_first_ == end_effector::LFoot)
  {
      footstep_list.insert(footstep_list.begin(), right_foot_stance_);
      footstep_list.insert(footstep_list.begin(), left_foot_stance_);
  }
  else
  {
      footstep_list.insert(footstep_list.begin(), left_foot_stance_);
      footstep_list.insert(footstep_list.begin(), right_foot_stance_);
  }
}

void NMPCHandler::_populateStepForward()
{
    _updateStartingStance(); // Update the starting foot locations of the robot

    FootStep left_footstep = left_foot_stance_;
    FootStep right_footstep = right_foot_stance_;
    FootStep mid_footstep = mid_foot_stance_;

    footstep_list = FootStep::GetFwdWalkFootStep(n_steps_, nominal_forward_step_, nominal_footwidth_, robot_side_first_, mid_footstep);

    if (robot_side_first_ == end_effector::LFoot)
    {
        footstep_list.insert(footstep_list.begin(), right_foot_stance_);
        footstep_list.insert(footstep_list.begin(), left_foot_stance_);
    }
    else
    {
        footstep_list.insert(footstep_list.begin(), left_foot_stance_);
        footstep_list.insert(footstep_list.begin(), right_foot_stance_);
    }

    // Assign iterators to keep the current foot position at the beginning of the walk
    init_it = footstep_list.begin();
    end_it = init_it + 2;
}

void NMPCHandler::_updateStartingStance()
{
  Eigen::Vector3d lfoot_pos = robot_->GetLinkIsometry(lfoot_id_).translation();
  Eigen::Quaterniond lfoot_ori(robot_->GetLinkIsometry(lfoot_id_).linear());
  left_foot_stance_.SetPosOriSide(lfoot_pos, lfoot_ori, end_effector::LFoot);

  Eigen::Vector3d rfoot_pos = robot_->GetLinkIsometry(rfoot_id_).translation();
  Eigen::Quaterniond rfoot_ori(robot_->GetLinkIsometry(rfoot_id_).linear());
  right_foot_stance_.SetPosOriSide(rfoot_pos, rfoot_ori, end_effector::RFoot);

  mid_foot_stance_.ComputeMidFoot(left_foot_stance_, right_foot_stance_,
                                  mid_foot_stance_);

}

void NMPCHandler::_resetIndexAndClearFootsteps()
{
  // Reset index and footstep list
  _resetStepIndex();
  footstep_list.clear();
}

void NMPCHandler::_resetStepIndex()
{
    current_footstep_idx_ = 0;
    footstep_list_index_ = 0;
}

void NMPCHandler::SetFootstepToPublish(const int count)
{
    count_ = count;

    if(!footstep_list.empty())
    {
        // Update the next footstep reference every T seconds
        /// Problem: simulation time is slower than actual time!
        if(count_ % int(T_ / ctrl_dt_) == 0 && footstep_list_index_ < int(footstep_list.size() - 4))
        {
            std::cout << "COUNT A: " << count_ << std::endl;
            std::cout << "Taking footsteps from " << footstep_list_index_ << " to " << footstep_list_index_ + 3 << " in a vector size of " << footstep_list.size() << std::endl;
            init_it = footstep_list.begin() + footstep_list_index_;
            end_it = footstep_list.begin() + footstep_list_index_ + 4;
            for (auto it = init_it; it != end_it; it++)
                std::cout << it->GetPos().transpose() << std::endl;
            is_new_ = true;
            footstep_list_index_ += 2;
        }
        else if(count_ % int(T_ / ctrl_dt_) == 0 && footstep_list_index_ >= int(footstep_list.size() - 4) && footstep_list_index_ < int(footstep_list.size() - 2))
        {
            std::cout << "COUNT B: " << count_ << std::endl;
            std::cout << "Taking footsteps from " << footstep_list_index_ << " to " << footstep_list_index_ + 1 << " in a vector size of " << footstep_list.size() << std::endl;
            init_it = footstep_list.begin() + footstep_list_index_;
            end_it = footstep_list.begin() + footstep_list_index_ + 2;
            for (auto it = init_it; it != end_it; it++)
                std::cout << it->GetPos().transpose() << std::endl;
            is_new_ = true;
            footstep_list_index_ += 2;
        }


        if (count_ % int(T_ / n_nodes_ / ctrl_dt_ + 0.5) == 0)
        {           
            if (count_ % int(T_ / ctrl_dt_) != 0)
            {
                std::cout << count_ << std::endl;
                is_new_ = false;
            }
            std::vector<FootStep> fs(init_it, end_it);
            footstep_to_publish_ = fs;
        }
    }
    else
    {
        if(count_ % int(T_ / n_nodes_ / ctrl_dt_) == 0)
        {
            is_new_ = false;
            if(first_visit_)
            {
                _updateStartingStance();
                std::vector<FootStep> fs{left_foot_stance_, right_foot_stance_};
                footstep_to_publish_ = fs;

                first_visit_ = false;
            }
        }
    }
}

void NMPCHandler::_GetMPCInputData()
{}

void NMPCHandler::_SendData()
{
    std::string encoded_msg;
    MPC_MSG::DracoState draco_state_msg;
    for (auto footstep : footstep_to_publish_)
    {
      MPC_MSG::Contact* contact_msg = draco_state_msg.add_contacts();
      if (footstep.GetFootSide() == end_effector::LFoot)
        contact_msg->set_name("l_foot_contact");
      else
        contact_msg->set_name("r_foot_contact");
      contact_msg->set_pos_x(footstep.GetPos()(0));
      contact_msg->set_pos_y(footstep.GetPos()(1));
      contact_msg->set_pos_z(footstep.GetPos()(2));
      contact_msg->set_ori_x(footstep.GetOrientation().x());
      contact_msg->set_ori_y(footstep.GetOrientation().y());
      contact_msg->set_ori_z(footstep.GetOrientation().z());
      contact_msg->set_ori_w(footstep.GetOrientation().w());
    }
    // send robot state
    Eigen::Vector3d com_pos = robot_->GetRobotComPos();
    Eigen::Vector3d com_vel = robot_->GetRobotComLinVel();
    for (int i = 0; i < com_pos.size(); i++)
    {
        draco_state_msg.add_com_pos(com_pos(i));
        draco_state_msg.add_com_vel(com_vel(i));
    }
    draco_state_msg.set_is_new(is_new_);
    draco_state_msg.SerializeToString(&encoded_msg);
    zmq::message_t zmq_msg(encoded_msg.size());
    memcpy ((void *) zmq_msg.data(), encoded_msg.c_str(), encoded_msg.size());
    publisher_->send(zmq_msg, zmq::send_flags::none);
}

void NMPCHandler::_GetMPCOutputData()
{
    // Retrieve MPC result
    zmq::message_t update_;
    subscriber_->recv(&update_, ZMQ_NOBLOCK);

    ZeroCopyInputStream* raw_input = new ArrayInputStream(update_.data(), update_.size());
    CodedInputStream* coded_input = new CodedInputStream(raw_input);

    std::string serialized_update;
    uint32_t serialized_size;

    coded_input->ReadVarint32(&serialized_size);
    coded_input->ReadString(&serialized_update, serialized_size);

    HORIZON_TO_PNC::MPCResult temp_res;
    temp_res.ParseFromArray(update_.data(), update_.size());
    if(temp_res.com_size() > 0)
    {
        solution_received_ = true;
        if (mpc_res_.com_size() > 0)
        {
            old_mpc_res_.CopyFrom(mpc_res_);
        }
        else
        {
            old_mpc_res_.CopyFrom(temp_res);
        }
        // Update old references for interpolation
        auto old_left_foot = _ConvertFoot(draco_link::l_foot_contact, old_mpc_res_, 1);
        old_lf_force_ = _ConvertFootForces(std::get<0>(old_left_foot), draco_link::l_foot_contact, old_mpc_res_, 1);

        auto old_right_foot = _ConvertFoot(draco_link::r_foot_contact, old_mpc_res_, 1);
        old_rf_force_ = _ConvertFootForces(std::get<0>(old_right_foot), draco_link::r_foot_contact, old_mpc_res_, 1);

        old_com_pos_ << old_mpc_res_.com(1).x(), old_mpc_res_.com(1).y(), old_mpc_res_.com(1).z();
        old_com_vel_ << old_mpc_res_.com_vel(1).xdot(), old_mpc_res_.com_vel(1).ydot(), old_mpc_res_.com_vel(1).zdot();
        old_com_acc_ << old_mpc_res_.com_acc(1).xddot(), old_mpc_res_.com_acc(1).yddot(), old_mpc_res_.com_acc(1).zddot();

        old_lf_pos_ = std::get<0>(old_left_foot);
        old_lf_vel_ = std::get<1>(old_left_foot);
        old_lf_acc_ = std::get<2>(old_left_foot);

        old_rf_pos_ = std::get<0>(old_right_foot);
        old_rf_vel_ = std::get<1>(old_right_foot);
        old_rf_acc_ = std::get<2>(old_right_foot);

        mpc_res_.CopyFrom(temp_res);
        interp_count_ = 0;

        auto right_foot_ref = _ConvertFoot(draco_link::r_foot_contact, mpc_res_, 1);
        auto new_rf_force = _ConvertFootForces(std::get<0>(right_foot_ref), draco_link::r_foot_contact, mpc_res_, 1);
    }
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> NMPCHandler::_ConvertFoot(int foot, HORIZON_TO_PNC::MPCResult mpc_res, int index)
{
    HORIZON_TO_PNC::FootPos foot_pos;
    HORIZON_TO_PNC::FootVel foot_vel;
    HORIZON_TO_PNC::FootAcc foot_acc;

    Eigen::Vector3d pos, vel, acc;
    pos.setZero();
    vel.setZero();
    acc.setZero();

    if (foot == draco_link::l_foot_contact)
    {
        foot_pos = mpc_res.left_foot_pos(index);
        foot_vel = mpc_res.left_foot_vel(index);
        foot_acc = mpc_res.left_foot_acc(index);
    }
    else if (foot == draco_link::r_foot_contact)
    {
        foot_pos = mpc_res.right_foot_pos(index);
        foot_vel = mpc_res.right_foot_vel(index);
        foot_acc = mpc_res.right_foot_acc(index);
    }
    else
        throw std::runtime_error("[NMPCHandler: convertFoot()]: Wrong link selected!");

    double avg_xdot = 0, avg_ydot = 0, avg_zdot = 0;
    double avg_xddot = 0, avg_yddot = 0, avg_zddot = 0;
    for (int i = 0; i < foot_pos.pos_size(); i++)
    {
        pos(0) += foot_pos.pos(i).x();      pos(1) += foot_pos.pos(i).y();      pos(2) += foot_pos.pos(i).z();
        vel(0) += foot_vel.vel(i).xdot();   vel(1) += foot_vel.vel(i).ydot();   vel(2) += foot_vel.vel(i).zdot();
        acc(0) += foot_acc.acc(i).xddot();  acc(1) += foot_acc.acc(i).yddot();  acc(2) += foot_acc.acc(i).zddot();
    }
    pos /= foot_pos.pos_size();
    vel /= foot_vel.vel_size();
    acc /= foot_acc.acc_size();

    return std::make_tuple(pos, vel, acc);
}

Eigen::Matrix<double, 6, 1> NMPCHandler::_ConvertFootForces(Eigen::Vector3d foot_center, int foot, HORIZON_TO_PNC::MPCResult mpc_res, int index)
{
    HORIZON_TO_PNC::ForceFoot force_foot;
    HORIZON_TO_PNC::FootPos foot_pos;
    Eigen::Matrix<double, 6, 1> wrench;
    wrench.setZero();

    if (foot == draco_link::l_foot_contact)
    {
        force_foot = mpc_res.force_left(index);
        foot_pos = mpc_res.left_foot_pos(index);

    }
    else if (foot == draco_link::r_foot_contact)
    {
        force_foot = mpc_res.force_right(index);
        foot_pos = mpc_res.right_foot_pos(index);
    }
    else
        throw std::runtime_error("[NMPCHandler: convertFootForces()]: Wrong link selected!");

    for (int i = 0; i < force_foot.force_size(); i++)
    {
        // TODO: CHECK!!
        Eigen::Vector3d arm = Eigen::Vector3d(foot_pos.pos(i).x(), foot_pos.pos(i).y(), foot_pos.pos(i).z()) - foot_center;
        wrench.tail<3>() += Eigen::Vector3d(force_foot.force(i).f_x(), force_foot.force(i).f_y(), force_foot.force(i).f_z());
        wrench.head<3>() += arm.cross(Eigen::Vector3d(force_foot.force(i).f_x(), force_foot.force(i).f_y(), force_foot.force(i).f_z()));
    }

    Eigen::Matrix<double, 6, 6> w_T_b;
    w_T_b.setZero();
    Eigen::Matrix3d w_R_b = robot_->GetLinkIsometry(foot).linear();
    w_T_b.block<3,3>(0,0) = w_R_b;
    w_T_b.block<3,3>(3,3) = w_R_b;

    wrench = w_T_b.transpose() * wrench;
    return wrench;
}

Eigen::VectorXd NMPCHandler::_LinearInterpolation(Eigen::VectorXd start, Eigen::VectorXd goal)
{
    int index = count_ % int(T_ / n_nodes_ / ctrl_dt_);
    Eigen::VectorXd res = start + (goal - start) / int(T_ / n_nodes_ / ctrl_dt_) * interp_count_;

    for (int i = 0; i < res.size(); i++)
    {
        if (goal(i) - start(i) > 0 && res(i) > goal(i))
            res(i) = goal(i);
        else if (goal(i) - start(i) < 0 && res(i) < goal(i))
            res(i) = goal(i);
    }

    return res;
}

bool NMPCHandler::UpdateDesired()
{
    // Update com
    Eigen::Vector3d com_pos_ref = _LinearInterpolation(old_com_pos_, Eigen::Vector3d(mpc_res_.com(1).x(), mpc_res_.com(1).y(), mpc_res_.com(1).z()));
    Eigen::Vector3d com_vel_ref = _LinearInterpolation(old_com_vel_, Eigen::Vector3d(mpc_res_.com_vel(1).xdot(), mpc_res_.com_vel(1).ydot(), mpc_res_.com_vel(1).zdot()));
    Eigen::Vector3d com_acc_ref = _LinearInterpolation(old_com_acc_, Eigen::Vector3d(mpc_res_.com_acc(1).xddot(), mpc_res_.com_acc(1).yddot(), mpc_res_.com_acc(1).zddot()));
    tci_container_->task_map_["com_task"]->UpdateDesired(com_pos_ref, com_vel_ref, com_acc_ref);

    logger_->add("unfiltered_com_pos_ref", Eigen::Vector3d(mpc_res_.com(1).x(), mpc_res_.com(1).y(), mpc_res_.com(1).z()));
    logger_->add("unfiltered_com_vel_ref", Eigen::Vector3d(mpc_res_.com_vel(1).xdot(), mpc_res_.com_vel(1).ydot(), mpc_res_.com_vel(1).zdot()));
    logger_->add("unfiltered_com_acc_ref", Eigen::Vector3d(mpc_res_.com_acc(1).xddot(), mpc_res_.com_acc(1).yddot(), mpc_res_.com_acc(1).zddot()));
    logger_->add("com_pos_ref", com_pos_ref);
    logger_->add("com_vel_ref", com_vel_ref);
    logger_->add("com_acc_ref", com_acc_ref);


    // Update torso orientation
    tci_container_->task_map_["torso_ori_task"]->UpdateDesired(Eigen::Vector4d(mpc_res_.ori(1).ori_x(), mpc_res_.ori(1).ori_y(), mpc_res_.ori(1).ori_z(), mpc_res_.ori(1).ori_w()),
                                                               Eigen::Vector3d(mpc_res_.omega(1).omega_x(), mpc_res_.omega(1).omega_y(), mpc_res_.omega(1).omega_z()),
                                                               Eigen::Vector3d(mpc_res_.omega_dot(1).omega_dot_x(), mpc_res_.omega_dot(1).omega_dot_y(), mpc_res_.omega_dot(1).omega_dot_z()));


    // Update left foot
    auto left_foot_ref = _ConvertFoot(draco_link::l_foot_contact, mpc_res_, 1);
    Eigen::Vector3d lf_pos_ref = _LinearInterpolation(old_lf_pos_, std::get<0>(left_foot_ref));
    Eigen::Vector3d lf_vel_ref = _LinearInterpolation(old_lf_vel_, std::get<1>(left_foot_ref));
    Eigen::Vector3d lf_acc_ref = _LinearInterpolation(old_lf_acc_, std::get<2>(left_foot_ref));
    tci_container_->task_map_["lf_pos_task"]->UpdateDesired(lf_pos_ref, lf_vel_ref, lf_acc_ref);

    tci_container_->task_map_["lf_ori_task"]->UpdateDesired(Eigen::Vector4d(0, 0, 0, 1),
                                                            Eigen::Vector3d(0, 0, 0),
                                                            Eigen::Vector3d(0, 0, 0));

    auto left_force_ref = _ConvertFootForces(std::get<0>(left_foot_ref), draco_link::l_foot_contact, mpc_res_, 1);
    auto left_force_ref_interpolated = _LinearInterpolation(old_lf_force_, left_force_ref);
    if (left_force_ref_interpolated[5] == 0)
        tci_container_->contact_map_["lf_contact"]->SetMaxFz(0);
    else
        tci_container_->contact_map_["lf_contact"]->SetMaxFz(1000);
    tci_container_->force_task_map_["lf_reaction_force_task"]->UpdateDesired(left_force_ref_interpolated);

    // Update right foot
    auto right_foot_ref = _ConvertFoot(draco_link::r_foot_contact, mpc_res_, 1);
    Eigen::Vector3d rf_pos_ref = _LinearInterpolation(old_rf_pos_, std::get<0>(right_foot_ref));
    Eigen::Vector3d rf_vel_ref = _LinearInterpolation(old_rf_vel_, std::get<1>(right_foot_ref));
    Eigen::Vector3d rf_acc_ref = _LinearInterpolation(old_rf_acc_, std::get<2>(right_foot_ref));

    tci_container_->task_map_["rf_pos_task"]->UpdateDesired(rf_pos_ref, rf_vel_ref, rf_acc_ref);



    tci_container_->task_map_["rf_ori_task"]->UpdateDesired(Eigen::Vector4d(0, 0, 0, 1),
                                                            Eigen::Vector3d(0, 0, 0),
                                                            Eigen::Vector3d(0, 0, 0));

    auto right_force_ref = _ConvertFootForces(std::get<0>(right_foot_ref), draco_link::r_foot_contact, mpc_res_, 1);
    auto right_force_ref_interpolated = _LinearInterpolation(old_rf_force_, right_force_ref);
    if (right_force_ref_interpolated[5] == 0)
        tci_container_->contact_map_["rf_contact"]->SetMaxFz(0);
    else
        tci_container_->contact_map_["rf_contact"]->SetMaxFz(1000);
    tci_container_->force_task_map_["rf_reaction_force_task"]->UpdateDesired(right_force_ref_interpolated);

//    auto init_dcm_pos = robot_->GetRobotComPos();
//    auto init_dcm_vel = robot_->GetRobotComLinVel();


    // Add to logger
    logger_->add("unfiltered_lf_pos_ref", std::get<0>(left_foot_ref));
    logger_->add("unfiltered_lf_vel_ref", std::get<1>(left_foot_ref));
    logger_->add("unfiltered_lf_acc_ref", std::get<2>(left_foot_ref));
    logger_->add("lf_pos_ref", lf_pos_ref);
    logger_->add("lf_vel_ref", lf_vel_ref);
    logger_->add("lf_acc_ref", lf_acc_ref);
    logger_->add("unfiltered_left_force_ref", left_force_ref);
    logger_->add("left_force_ref", _LinearInterpolation(old_lf_force_, left_force_ref));
    logger_->add("unfiltered_rf_pos_ref", std::get<0>(right_foot_ref));
    logger_->add("unfiltered_rf_vel_ref", std::get<1>(right_foot_ref));
    logger_->add("unfiltered_rf_acc_ref", std::get<2>(right_foot_ref));
    logger_->add("rf_pos_ref", rf_pos_ref);
    logger_->add("rf_vel_ref", rf_vel_ref);
    logger_->add("rf_acc_ref", rf_acc_ref);
    logger_->add("unfiltered_right_force_ref", right_force_ref);
    logger_->add("right_force_ref", _LinearInterpolation(old_rf_force_, right_force_ref));
    logger_->add("time", count_ * ctrl_dt_);

    interp_count_++;

    return true;
}


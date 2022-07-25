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
rfoot_id_(rfoot_id)
{
    util::PrettyConstructor(2, "NMPCHandler");

    _resetStepIndex();

    // At the moment, the MPC handles a walk that always starts with the left foot
    robot_side_first_ = end_effector::LFoot;
    solution_received_ = false;
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
}

void NMPCHandler::SetFootstepToPublish(const int count)
{
    if(!footstep_list.empty())
    {
        bool is_new = true;
        // Update the next footstep reference every T = 1s
        /// Problem: simulation time is slower than actual time!
        if(count % 800 == 0 && footstep_list_index_ < int(footstep_list.size() - 4))
        {
            std::cout << "COUNT A: " << count << std::endl;
            footstep_list_index_ += 2;
            std::cout << "Taking footsteps from " << footstep_list_index_ << " to " << footstep_list_index_ + 3 << " in a vector size of " << footstep_list.size() << std::endl;
            init_it = footstep_list.begin() + footstep_list_index_;
            end_it = footstep_list.begin() + footstep_list_index_ + 4;
            for (auto it = init_it; it != end_it; it++)
                std::cout << it->GetPos().transpose() << std::endl;
        }
        else if(count % 800 == 0 && footstep_list_index_ >= int(footstep_list.size() - 4) && footstep_list_index_ < int(footstep_list.size() - 2))
        {
            std::cout << "COUNT B: " << count << std::endl;
            footstep_list_index_ += 2;
            std::cout << "Taking footsteps from " << footstep_list_index_ << " to " << footstep_list_index_ + 1 << " in a vector size of " << footstep_list.size() << std::endl;
            init_it = footstep_list.begin() + footstep_list_index_;
            end_it = footstep_list.begin() + footstep_list_index_ + 2;
            for (auto it = init_it; it != end_it; it++)
                std::cout << it->GetPos().transpose() << std::endl;
        }

        // Create Protobuf message
        if (count % 40 == 0) // && footstep_list_index_ >= 0)
        {
            if (count % 800 != 0)
                is_new = false;
            std::vector<FootStep> fs(init_it, end_it);
            footstep_to_publish_ = fs;
        }
    }
    else
    {
        if(count % 40 == 0)
        {
            FootStep initial_footstep_left, initial_footstep_right;
            initial_footstep_left.SetLeftSide();
            Eigen::Quaternion<double> q_left(robot_->GetLinkIsometry(draco_link::l_foot_contact).linear());
            initial_footstep_left.SetPosOri(robot_->GetLinkIsometry(draco_link::l_foot_contact).translation(), q_left);
            initial_footstep_right.SetRightSide();
            Eigen::Quaternion<double> q_right(robot_->GetLinkIsometry(draco_link::r_foot_contact).linear());
            initial_footstep_right.SetPosOri(robot_->GetLinkIsometry(draco_link::r_foot_contact).translation(), q_right);

            std::vector<FootStep> fs{initial_footstep_left, initial_footstep_right};
            footstep_to_publish_ = fs;
        }
    }
}

void NMPCHandler::IsNew(const bool is_new)
{
    is_new_ = is_new;
}

void NMPCHandler::_GetMPCInputData()
{}

void NMPCHandler::_SendData()
{
    std::string encoded_msg;
    MPC_MSG::DracoState draco_state_msg;
    for (auto footstep : footstep_to_publish_)
    {
      MPC_MSG::Contact* contact_msg =draco_state_msg.add_contacts();
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
    Eigen::VectorXd q = robot_->GetQ();
    Eigen::VectorXd qdot = robot_->GetQdot();
    for (int i = 0; i < q.size(); i++)
    {
        draco_state_msg.add_joint_positions(q(i));
        draco_state_msg.add_joint_velocities(qdot(i));
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
        mpc_res_.CopyFrom(temp_res);
    }
}

std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> NMPCHandler::_ConvertFoot(int foot)
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
        foot_pos = mpc_res_.left_foot_pos(0);
        foot_vel = mpc_res_.left_foot_vel(0);
        foot_acc = mpc_res_.left_foot_acc(0);
    }
    else if (foot == draco_link::r_foot_contact)
    {
        foot_pos = mpc_res_.right_foot_pos(0);
        foot_vel = mpc_res_.right_foot_vel(0);
        foot_acc = mpc_res_.right_foot_acc(0);
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

Eigen::Matrix<double, 6, 1> NMPCHandler::_ConvertFootForces(Eigen::Vector3d foot_center, int foot)
{
    HORIZON_TO_PNC::ForceFoot force_foot;
    HORIZON_TO_PNC::FootPos foot_pos;
    Eigen::Matrix<double, 6, 1> wrench;
    wrench.setZero();

    if (foot == draco_link::l_foot_contact)
    {
        force_foot = mpc_res_.force_left(0);
        foot_pos = mpc_res_.left_foot_pos(0);
    }
    else if (foot == draco_link::r_foot_contact)
    {
        force_foot = mpc_res_.force_right(0);
        foot_pos = mpc_res_.right_foot_pos(0);
    }
    else
        throw std::runtime_error("[NMPCHandler: convertFootForces()]: Wrong link selected!");

    for (int i = 0; i < force_foot.force_size(); i++)
    {
        Eigen::Vector3d arm = Eigen::Vector3d(foot_pos.pos(i).x(), foot_pos.pos(i).y(), foot_pos.pos(i).z()) - foot_center;
        wrench.block<3, 1>(0, 0) += Eigen::Vector3d(force_foot.force(i).f_x(), force_foot.force(i).f_y(), force_foot.force(i).f_z());
        wrench.block<3, 1>(3, 0) += arm.cross(Eigen::Vector3d(force_foot.force(i).f_x(), force_foot.force(i).f_y(), force_foot.force(i).f_z()));
    }

    return wrench;
}

bool NMPCHandler::UpdateDesired()
{
//    std::cout << "starting updating" << std::endl;
    // Update com
    tci_container_->task_map_["com_task"]->UpdateDesired(Eigen::Vector3d(mpc_res_.com(0).x(), mpc_res_.com(0).y(), mpc_res_.com(0).z()),
                                                         Eigen::Vector3d(mpc_res_.com_vel(0).xdot(), mpc_res_.com_vel(0).ydot(), mpc_res_.com_vel(0).zdot()),
                                                         Eigen::Vector3d(mpc_res_.com_acc(0).xddot(), mpc_res_.com_acc(0).yddot(), mpc_res_.com_acc(0).zddot()));

//    std::cout << "com updated: " << mpc_res_.com(0).x() << ", " << mpc_res_.com(0).y() << ", " << mpc_res_.com(0).z() << std::endl;

    // Update left foot
    auto left_foot_ref = _ConvertFoot(draco_link::l_foot_contact);
//    std::cout << "left foot pose converted" << std::endl;
    tci_container_->task_map_["lf_pos_task"]->UpdateDesired(std::get<0>(left_foot_ref),
                                                            std::get<1>(left_foot_ref),
                                                            std::get<2>(left_foot_ref));
//    std::cout << "left foot pose updated: " << std::get<0>(left_foot_ref).transpose() << std::endl;

    auto left_force_ref = _ConvertFootForces(std::get<0>(left_foot_ref), draco_link::l_foot_contact);
//    std::cout << "left foot force converted" << std::endl;
    tci_container_->force_task_map_["lf_reaction_force_task"]->UpdateDesired(left_force_ref);
//    std::cout << "left foot force updated: " << left_force_ref.transpose() << std::endl;

    // Update right foot
    auto right_foot_ref = _ConvertFoot(draco_link::r_foot_contact);
//    std::cout << "right foot pose converted" << std::endl;
    tci_container_->task_map_["rf_pos_task"]->UpdateDesired(std::get<0>(right_foot_ref),
                                                            std::get<1>(right_foot_ref),
                                                            std::get<2>(right_foot_ref));
//    std::cout << "right foot pose updated: " << std::get<0>(right_foot_ref).transpose() << std::endl;

    auto right_force_ref = _ConvertFootForces(std::get<0>(right_foot_ref), draco_link::r_foot_contact);
//    std::cout << "right foot force converted" << std::endl;
    tci_container_->force_task_map_["rf_reaction_force_task"]->UpdateDesired(right_force_ref);
//    std::cout << "right foot force updated: " << right_force_ref.transpose() << std::endl;
    return true;
}


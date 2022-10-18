#include "controller/model_predictive_controller/nmpc/nmpc_handler.hpp"
#include "controller/robot_system/pinocchio_robot_system.hpp"
#include "controller/draco_controller/draco_definition.hpp"
#include "controller/whole_body_controller/tci_container.hpp"
#include "controller/whole_body_controller/basic_task.hpp"
#include "controller/whole_body_controller/basic_contact.hpp"
#include "controller/whole_body_controller/force_task.hpp"

#include <muvt_core/environment/contact/vertex_contact.h>
#include <muvt_core/environment/contact/edge_collision.h>
#include <muvt_core/environment/contact/edge_relative_pose.h>
#include <muvt_core/environment/contact/edge_steering.h>
#include <muvt_core/environment/contact/edge_task.h>

#include <tuple>

#define GRAVITY 9.81

using namespace google::protobuf::io;

static std::default_random_engine randGenerator;
static std::uniform_real_distribution<double> randDistribution(-0.05, 0.05);

NMPCHandler::NMPCHandler(PinocchioRobotSystem *robot, TCIContainer* tci_container, int lfoot_id, int rfoot_id):
MPCHandler(robot),
tci_container_(tci_container),
lfoot_id_(lfoot_id),
rfoot_id_(rfoot_id),
first_visit_(true),
_obstacle(Eigen::Vector3d(0.7, 1.0, 0.0)),
optimizer_(new Muvt::HyperGraph::OptimizerContact())
{

    auto a = std::chrono::system_clock::now();
    time_t b = std::chrono::system_clock::to_time_t(a);
    randGenerator.seed(b);

    util::PrettyConstructor(2, "NMPCHandler");

    _resetStepIndex();

    // At the moment, the MPC handles a walk that always starts with the left foot
    robot_side_first_ = end_effector::LFoot;
    solution_received_ = false;

    T_ = 1.5;
    n_nodes_ = 20;
    ctrl_dt_ = 0.00125;
    is_new_ = false;

    // Initialize vectors for linear interpolation
    old_lf_force_.setZero(6);   old_rf_force_.setZero(6);

    old_com_pos_.setZero(3);    old_com_vel_.setZero(3);    old_com_acc_.setZero(3);
    old_base_ori_.setZero(4);   old_base_vel_.setZero(3);   old_base_acc_.setZero(3);
    old_lf_pos_.setZero(3);     old_lf_vel_.setZero(3);     old_lf_acc_.setZero(3);
    old_rf_pos_.setZero(3);     old_rf_vel_.setZero(3);     old_rf_acc_.setZero(3);

    interp_count_ = 0;
    c_ = 0;

    logger_ = XBot:: MatLogger2::MakeLogger("/tmp/tasks_references");
}

void NMPCHandler::paramInitialization(const YAML::Node &node)
{
  // void setCoMHeight(double z_vrp_in); // Sets the desired CoM Height
  // Load Custom Params ----------------------------------
  try {
    // Load DCM Parameters
    util::ReadParameter(node, "com_height", nominal_com_height_);

    // Load Walking Primitives Parameters
    util::ReadParameter(node, "nominal_footwidth", nominal_footwidth_);
    util::ReadParameter(node, "nominal_forward_step", nominal_forward_step_);
    util::ReadParameter(node, "nominal_backward_step", nominal_backward_step_);
    util::ReadParameter(node, "nominal_turn_radians", nominal_turn_radians_);
    util::ReadParameter(node, "nominal_strafe_distance", nominal_strafe_distance_);
    util::ReadParameter(node, "n_steps", n_steps_);
    util::ReadParameter(node, "time_horizon_length", T_);

  } catch (std::runtime_error &e) {
    std::cout << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl
              << std::endl;
    exit(0);
  }
}

void NMPCHandler::SetDCMPos(Eigen::Vector3d dcm)
{
    dcm_ = dcm;
}

void NMPCHandler::walkInPlace()
{
    _resetIndexAndClearFootsteps();
    _populateStepInPlace();
    _generateDCMTrajectory();
}

void NMPCHandler::walkForward()
{
    _resetIndexAndClearFootsteps();
    _populateStepForward();
    _generateDCMTrajectory();
}

void NMPCHandler::walkSide(bool left_side)
{
    _resetIndexAndClearFootsteps();
    _populateSideWalk(left_side);
    _generateDCMTrajectory();
}

void NMPCHandler::_generateDCMTrajectory()
{
    // define useful quantities
    double T_step = T_/n_nodes_ * 10;
    double dt = T_/n_nodes_;
    double omega = std::sqrt(GRAVITY / nominal_com_height_);

    // generate a DCM and a Com trajectory on n_nodes, assuming the ZMP is in the center of each foot and the
    // DCM terminates on the final foot
    std::vector<Eigen::Vector3d> dcm_pts(footstep_list.size());
    std::vector<Eigen::Vector3d> dcm_trj;

    dcm_pts.back() = footstep_list.back().GetPos();
    for (int i = footstep_list.size() - 2; i > 0; i--)
    {
        Eigen::VectorXd new_dcm = footstep_list[i].GetPos() + (dcm_pts[i+1] - footstep_list[i].GetPos()) / std::exp(omega * T_step);
        dcm_pts[i] = new_dcm;
    }
    dcm_pts[0] = mid_foot_stance_.GetPos();

    // compute DCM trajectory
    double time = dt;
    for (int j = 1; j < dcm_pts.size() - 2; j+=2)
    {
        time = 0;
        for (int i = 0; i <= 10; i++)
        {
            dcm_trj.push_back(std::exp(omega * time) * dcm_pts[j] + (1 - std::exp(omega * time)) * footstep_list[j].GetPos());
            time += dt;
        }

        time = 0;
        for (int i = 11; i <= 21; i++)
        {
            dcm_trj.push_back(std::exp(omega * time) * dcm_pts[j+1] + (1 - std::exp(omega * time)) * footstep_list[j+1].GetPos());
            time += dt;
        }

    }
    com_trj_.resize(dcm_trj.size());
    com_dot_trj_.resize(dcm_trj.size());

    // compute com trajectory from dcm trajectory
    com_trj_[0] = tci_container_->task_map_["com_task"]->CurrentPos();
    com_trj_[0][2] = nominal_com_height_;
    com_dot_trj_[0] = tci_container_->task_map_["com_task"]->CurrentVel();
    for (int i = 1; i < dcm_trj.size(); i++)
    {
        com_dot_trj_[i] = (-omega * com_trj_[i-1] + omega * dcm_trj[i-1]);
        com_dot_trj_[i][2] = 0;
        com_trj_[i] = dt * com_dot_trj_[i] + com_trj_[i-1];
    }
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

    init_local_planner(); // Initialize local planner

    // Assign iterators to keep the current foot position at the beginning of the walk
    init_it = footstep_list.begin();
    end_it = init_it + 2;
}

void NMPCHandler::_populateSideWalk(bool left_side)
{
    _updateStartingStance();

    FootStep left_footstep = left_foot_stance_;
    FootStep right_footstep = right_foot_stance_;
    FootStep mid_footstep = mid_foot_stance_;

    if (!left_side)
        nominal_footwidth_ *= -1;

    footstep_list = FootStep::GetStrafeFootStep(n_steps_, nominal_strafe_distance_, nominal_footwidth_, mid_foot_stance_);

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

void NMPCHandler::init_local_planner()
{
    if (footstep_list.empty())
        throw std::runtime_error("footstep_list is empty and the local planner cannot be initialized!");

    optimizer_->clear();

//    int index = 2;
    int index = 0;
    std::vector<g2o::OptimizableGraph::Vertex*> vertices;

    for (auto footstep : footstep_list)
    {
        Muvt::HyperGraph::Contact contact;
        contact.state.pose.translation() = footstep.GetPos();
        contact.state.pose.linear() = footstep.GetOrientation().toRotationMatrix();
        if (footstep.GetFootSide() == end_effector::LFoot)
            contact.setDistalLink("l_foot");
        else if (footstep.GetFootSide() == end_effector::RFoot)
            contact.setDistalLink("r_foot");

        Muvt::HyperGraph::VertexContact* v = new Muvt::HyperGraph::VertexContact();
        v->setId(index);
        v->setEstimate(contact);
        if(index == 0 || index == 1)
            v->setFixed(true);

        auto vertex = dynamic_cast<g2o::OptimizableGraph::Vertex*>(v);
        vertices.push_back(vertex);
        index++;
    }

    optimizer_->setVertices(vertices);

    std::vector<g2o::OptimizableGraph::Edge*> edges;

    // EdgeTask for the last two vertices only
    for (int i = vertices.size() - 2; i < vertices.size(); i++)
    {
        Muvt::HyperGraph::EdgeTask* edge = new Muvt::HyperGraph::EdgeTask();
        Eigen::Matrix3d info = Eigen::Matrix3d::Identity();
        info *= 10;
        edge->setInformation(info);
        auto v = dynamic_cast<Muvt::HyperGraph::VertexContact*>(vertices[i]);
        edge->setReference(v->estimate().state.pose.translation());
        edge->vertices()[0] = vertices[i];
        auto e = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);
        edges.push_back(e);
    }

    for (int i = 0; i < vertices.size(); i++)
    {
      int m = 5;
      Muvt::HyperGraph::EdgeCollision* edge = new Muvt::HyperGraph::EdgeCollision();
      Eigen::MatrixXd info(1, 1);
      info.setIdentity(); info *= 100;
      edge->setInformation(info);
      Eigen::Vector3d obstacle = Eigen::Vector3d(1.3, 0.0, 0.0);
      edge->setObstacles(obstacle);
      edge->vertices()[0] = vertices[i];
      auto e = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);
      edges.push_back(e);
    }

    for (int i = 0; i < vertices.size() - 1; i++)
    {
        Muvt::HyperGraph::EdgeRelativePose* edge_succ = new Muvt::HyperGraph::EdgeRelativePose();
        Eigen::MatrixXd info_succ(3, 3);
        info_succ.setIdentity(); info_succ(2,2) *= 100;
        edge_succ->setInformation(info_succ);
        edge_succ->setStepSize(nominal_forward_step_);
        edge_succ->vertices()[0] = vertices[i];
        edge_succ->vertices()[1] = vertices[i+1];
        auto e = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge_succ);
        edges.push_back(e);
     }

    for (unsigned int i = 2; i < vertices.size(); i++)
    {
        Muvt::HyperGraph::EdgeSteering* edge = new Muvt::HyperGraph::EdgeSteering();
        Eigen::MatrixXd info(3, 3);
        info.setIdentity(); info *= 10;
        edge->setInformation(info);
        edge->setPreviousContact(vertices[i-2]);
        edge->vertices()[0] = vertices[i];
        auto e = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);
        edges.push_back(e);
    }

    optimizer_->setEdges(edges);
    optimizer_->update();

    localPlan();
}

void NMPCHandler::localPlan()
{
    auto edges = optimizer_->getEdges();
    for (auto edge : edges)
    {
        if (auto e = dynamic_cast<Muvt::HyperGraph::EdgeCollision*>(edge); e != nullptr)
        {
            e->setObstacles(_obstacle);
        }
    }
    auto tic = std::chrono::high_resolution_clock::now();
    optimizer_->solve();
    auto toc = std::chrono::high_resolution_clock::now();
    std::chrono::duration<float> fsec = toc - tic;
    logger_->add("time_local_planner", fsec.count());

    std::vector<Muvt::HyperGraph::Contact> solution;
    optimizer_->getFootsteps(solution);

    for (int i = 0; i < solution.size(); i++)
    {
        Eigen::Quaterniond q(solution[i].state.pose.linear());
        footstep_list[i].SetPosOri(solution[i].state.pose.translation(), q);
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
    footstep_list_index_ = 0;
    init_com_trj_index_ = 0;
}

Eigen::Vector3d NMPCHandler::_obstacle_trajectory(int count, double T, Eigen::Vector3d start, Eigen::Vector3d goal)
{
    Eigen::Vector3d obstacle;
    double time = (count - count_init_) * ctrl_dt_;

    Eigen::Vector3d a = (start - goal) * 2 / std::pow(T, 3);
    Eigen::Vector3d b = -3*a*T / 2;
    Eigen::Vector3d c = Eigen::Vector3d::Zero();
    Eigen::Vector3d d = start;

    obstacle = a * std::pow(time, 3) + b * std::pow(time, 2) + c * time + d;

    return obstacle;
}

void NMPCHandler::SetFootstepToPublish(const int count)
{
    count_ = count;

    if(!footstep_list.empty())
    {
        localPlan();
        // Update the next footstep reference every T seconds
        /// Problem: simulation time is slower than actual time!
        if(count_ % int(T_ / ctrl_dt_ + 0.5) == 0 && footstep_list_index_ < int(footstep_list.size() - 2))
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
        else if(count_ % int(T_ / ctrl_dt_ + 0.5) == 0 && footstep_list_index_ >= int(footstep_list.size() - 2)&& footstep_list_index_ < int(footstep_list.size() - 1))
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
            if (count_ % int(T_ / ctrl_dt_ + 0.5) != 0)
            {
                is_new_ = false;
            }
            std::vector<FootStep> fs(init_it, end_it);
            footstep_to_publish_ = fs;

            // Update com and com_dot sub-vector to be sent as the initial guess
            if (footstep_list_index_ > 2)
            {
                com_trj_sv_.clear();
                com_dot_trj_sv_.clear();
                auto it_start = com_trj_.begin() + init_com_trj_index_;
                auto it_end = com_trj_.begin() + init_com_trj_index_ + n_nodes_ + 1;
                while(it_start != com_trj_.end() && it_start != it_end)
                {
                    com_trj_sv_.push_back(*it_start);
                    it_start++;
                }

                it_start = com_dot_trj_.begin() + init_com_trj_index_;
                it_end = com_dot_trj_.begin() + init_com_trj_index_ + n_nodes_ + 1;

                while(it_start != com_dot_trj_.end() && it_start != it_end)
                {
                    com_dot_trj_sv_.push_back(*it_start);
                    it_start++;
                }

                int res_size = n_nodes_ + 1 - com_trj_sv_.size();
                while (res_size > 0)
                {
                    com_trj_sv_.push_back(*(com_trj_.end() - 1));
                    com_dot_trj_sv_.push_back(Eigen::Vector3d(0., 0., 0.));
                    res_size--;
                }

                if (init_com_trj_index_ < com_trj_.size())
                    init_com_trj_index_++;

            }
        }
    }
    else
    {
        if(count_ % int(T_ / n_nodes_ / ctrl_dt_ + 0.5) == 0)
        {
            is_new_ = false;
            if(first_visit_)
            {
                _updateStartingStance();
                std::vector<FootStep> fs{left_foot_stance_, right_foot_stance_};
                footstep_to_publish_ = fs;
                first_visit_ = false;
            }
//            else
//            {
//                if (count_ % int(T_ / ctrl_dt_) == 0)
//                {
//                    _updateStartingStance();
//                    std::vector<FootStep> fs{left_foot_stance_, right_foot_stance_};
//                    footstep_to_publish_ = fs;
//                }
//            }
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
    /////////////////////////
    // REMOVE WHEN FINISHED
    for (auto footstep : footstep_list)
    {
      MPC_MSG::Contact* contact_msg = draco_state_msg.add_footstep_list();
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
    MPC_MSG::ComPos obstacle = draco_state_msg.obstacle();
    obstacle.set_x(_obstacle(0));
    obstacle.set_y(_obstacle(1));
    obstacle.set_z(_obstacle(2));
    //////////////////////////
    // send robot state
    Eigen::Vector3d com_pos = tci_container_->task_map_["com_task"]->CurrentPos();
    Eigen::Vector3d com_vel = tci_container_->task_map_["com_task"]->CurrentVel();
    Eigen::Vector4d torso_ori = tci_container_->task_map_["torso_ori_task"]->CurrentPos();
    Eigen::Vector3d torso_vel = tci_container_->task_map_["torso_ori_task"]->CurrentVel();
    for (int i = 0; i < com_pos.size(); i++)
    {
        draco_state_msg.add_com_pos(com_pos(i));
        draco_state_msg.add_com_vel(com_vel(i));
        draco_state_msg.add_base_ori(torso_ori(i));
        draco_state_msg.add_base_vel(torso_vel(i));
    }
    draco_state_msg.add_base_ori(torso_ori(3));
    draco_state_msg.set_count(count_);
    draco_state_msg.set_is_new(is_new_);
    draco_state_msg.set_footstep_index(footstep_list_index_);

    if (!com_trj_sv_.empty())
    {
        init_com_pos_.clear();
        init_com_vel_.clear();

        for (int i = 0; i < n_nodes_ + 1; i++)
        {
            MPC_MSG::ComPos com_pos;
            com_pos.set_x(com_trj_sv_[i](0));
            com_pos.set_y(com_trj_sv_[i](1));
            com_pos.set_z(com_trj_sv_[i](2));
            init_com_pos_.push_back(com_pos);

            MPC_MSG::ComVel com_vel;
            com_vel.set_xdot(com_dot_trj_sv_[i](0));
            com_vel.set_ydot(com_dot_trj_sv_[i](1));
            com_vel.set_zdot(com_dot_trj_sv_[i](2));
            init_com_vel_.push_back(com_vel);
        }

        for (int i = 0; i < init_com_pos_.size(); i++)
        {
            auto com_pos = draco_state_msg.add_init_com_pos();
            com_pos->CopyFrom(init_com_pos_[i]);
            auto com_vel = draco_state_msg.add_init_com_vel();
            com_vel->CopyFrom(init_com_vel_[i]);
        }
    }

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
//    c_++;
    if(temp_res.com_size() > 0)
    {
//        std::cout << c_ << std::endl;
//        c_ = 0;
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

        old_base_ori_ << old_mpc_res_.ori(1).x(), old_mpc_res_.ori(1).y(), old_mpc_res_.ori(1).z(), old_mpc_res_.ori(1).w();
        old_base_vel_ << old_mpc_res_.omega(1).x(), old_mpc_res_.omega(1).y(), old_mpc_res_.omega(1).z();

        old_lf_pos_ = std::get<0>(old_left_foot);
        old_lf_vel_ = std::get<1>(old_left_foot);
        old_lf_acc_ = std::get<2>(old_left_foot);

        old_rf_pos_ = std::get<0>(old_right_foot);
        old_rf_vel_ = std::get<1>(old_right_foot);
        old_rf_acc_ = std::get<2>(old_right_foot);

        mpc_res_.CopyFrom(temp_res);
        interp_count_ = 0;

//        std::cout << "count_: " << count_ << " - from [" << old_com_pos_.transpose() << "] to [" << mpc_res_.com(1).x() << ", " << mpc_res_.com(1).y() << ", " << mpc_res_.com(1).z() << "]" << std::endl;

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

std::tuple<Eigen::Vector4d, Eigen::Vector3d, Eigen::Vector3d> NMPCHandler::_ConvertFootOri(int foot, HORIZON_TO_PNC::MPCResult mpc_res)
{
    HORIZON_TO_PNC::FootPos foot_pos;
    HORIZON_TO_PNC::FootVel foot_vel;
    HORIZON_TO_PNC::FootAcc foot_acc;

    double alpha; // Angle offset between the parallel to the local x of the foot and the line bassing through the two points of each foot

    if (foot == draco_link::l_foot_contact)
    {
        foot_pos = mpc_res.left_foot_pos(1);
        foot_vel = mpc_res.left_foot_vel(1);
        foot_acc = mpc_res.left_foot_acc(1);
        alpha = -0.4636;
    }
    else if (foot == draco_link::r_foot_contact)
    {
        foot_pos = mpc_res.right_foot_pos(1);
        foot_vel = mpc_res.right_foot_vel(1);
        foot_acc = mpc_res.right_foot_acc(1);
        alpha = 0.4636;
    }

    // Orientation
    Eigen::Vector3d line = Eigen::Vector3d(foot_pos.pos(0).x() - foot_pos.pos(1).x(), foot_pos.pos(0).y() - foot_pos.pos(1).y(), foot_pos.pos(0).z() - foot_pos.pos(1).z());
    Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();
    rot.col(0) = line;  rot.col(0).normalize();
    rot.col(2) = Eigen::Vector3d(0, 0, 1);
    rot.col(1) = rot.col(2).cross(rot.col(0));
    Eigen::Matrix3d rot_offset;
    rot_offset << cos(alpha), -sin(alpha), 0, sin(alpha), cos(alpha), 0, 0, 0, 1;
    Eigen::Quaternion<double> quat(rot * rot_offset);
    Eigen::Vector4d ori = Eigen::Vector4d(quat.coeffs().x(), quat.coeffs().y(), quat.coeffs().z(), quat.coeffs().w());

    // Angular velocity
    Eigen::Vector3d omega;
    if (foot == draco_link::l_foot_contact)
    {
        omega = Eigen::Vector3d(0.04, 0.02, 0.2).cross(Eigen::Vector3d(foot_vel.vel(0).xdot(), foot_vel.vel(0).ydot(), foot_vel.vel(0).zdot())) +
                Eigen::Vector3d(-0.04, -0.02, 0.2).cross(Eigen::Vector3d(foot_vel.vel(1).xdot(), foot_vel.vel(1).ydot(), foot_vel.vel(1).zdot()));
    }
    else
    {
        omega = Eigen::Vector3d(0.04, -0.02, 0.2).cross(Eigen::Vector3d(foot_vel.vel(0).xdot(), foot_vel.vel(0).ydot(), foot_vel.vel(0).zdot())) +
                Eigen::Vector3d(-0.04, 0.02, 0.2).cross(Eigen::Vector3d(foot_vel.vel(1).xdot(), foot_vel.vel(1).ydot(), foot_vel.vel(1).zdot()));
    }

    // Angular acceleration
    Eigen::Vector3d omega_dot;
    if (foot == draco_link::l_foot_contact)
    {
        omega_dot = Eigen::Vector3d(0.04, 0.02, 0.2).cross(Eigen::Vector3d(foot_acc.acc(0).xddot(), foot_acc.acc(0).yddot(), foot_acc.acc(0).zddot())) +
                Eigen::Vector3d(-0.04, -0.02, 0.2).cross(Eigen::Vector3d(foot_acc.acc(1).xddot(), foot_acc.acc(1).yddot(), foot_acc.acc(1).zddot()));
    }
    else
    {
        omega_dot = Eigen::Vector3d(0.04, -0.02, 0.2).cross(Eigen::Vector3d(foot_acc.acc(0).xddot(), foot_acc.acc(0).yddot(), foot_acc.acc(0).zddot())) +
                Eigen::Vector3d(-0.04, 0.02, 0.2).cross(Eigen::Vector3d(foot_acc.acc(1).xddot(), foot_acc.acc(1).yddot(), foot_acc.acc(1).zddot()));
    }

    return std::make_tuple(ori, omega, omega_dot);
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

    // Update torso orientation
//    tci_container_->task_map_["torso_ori_task"]->UpdateDesired(Eigen::Vector4d(mpc_res_.ori(1).x(), mpc_res_.ori(1).y(), mpc_res_.ori(1).z(), mpc_res_.ori(1).w()),
//                                                               Eigen::Vector3d(mpc_res_.omega(1).x(), mpc_res_.omega(1).y(), mpc_res_.omega(1).z()),
//                                                               Eigen::Vector3d(mpc_res_.omega_dot(1).x(), mpc_res_.omega_dot(1).y(), mpc_res_.omega_dot(1).z()));
    tci_container_->task_map_["torso_ori_task"]->UpdateDesired(Eigen::Vector4d(0, 0, 0 ,1),
                                                               Eigen::Vector3d(0, 0, 0),
                                                               Eigen::Vector3d(0, 0, 0));

    // Update left foot
    auto left_foot_ref = _ConvertFoot(draco_link::l_foot_contact, mpc_res_, 1);
    Eigen::Vector3d lf_pos_ref = _LinearInterpolation(old_lf_pos_, std::get<0>(left_foot_ref));
    Eigen::Vector3d lf_vel_ref = _LinearInterpolation(old_lf_vel_, std::get<1>(left_foot_ref));
    Eigen::Vector3d lf_acc_ref = _LinearInterpolation(old_lf_acc_, std::get<2>(left_foot_ref));
    tci_container_->task_map_["lf_pos_task"]->UpdateDesired(lf_pos_ref, lf_vel_ref, lf_acc_ref);


//    tci_container_->task_map_["lf_ori_task"]->UpdateDesired(Eigen::Vector4d(0, 0, 0, 1),
//                                                            Eigen::Vector3d(0, 0, 0),
//                                                            Eigen::Vector3d(0, 0, 0));

    auto left_foot_ori_ref = _ConvertFootOri(draco_link::l_foot_contact, mpc_res_);
    tci_container_->task_map_["lf_ori_task"]->UpdateDesired(std::get<0>(left_foot_ori_ref),
                                                            std::get<1>(left_foot_ori_ref),
                                                            std::get<2>(left_foot_ori_ref));

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

//    tci_container_->task_map_["rf_ori_task"]->UpdateDesired(Eigen::Vector4d(0, 0, 0, 1),
//                                                            Eigen::Vector3d(0, 0, 0),
//                                                            Eigen::Vector3d(0, 0, 0));

    auto right_foot_ori_ref = _ConvertFootOri(draco_link::r_foot_contact, mpc_res_);
    tci_container_->task_map_["rf_ori_task"]->UpdateDesired(std::get<0>(right_foot_ori_ref),
                                                            std::get<1>(right_foot_ori_ref),
                                                            std::get<2>(right_foot_ori_ref));

    auto right_force_ref = _ConvertFootForces(std::get<0>(right_foot_ref), draco_link::r_foot_contact, mpc_res_, 1);
    auto right_force_ref_interpolated = _LinearInterpolation(old_rf_force_, right_force_ref);
    if (right_force_ref_interpolated[5] == 0)
        tci_container_->contact_map_["rf_contact"]->SetMaxFz(0);
    else
        tci_container_->contact_map_["rf_contact"]->SetMaxFz(1000);
    tci_container_->force_task_map_["rf_reaction_force_task"]->UpdateDesired(right_force_ref_interpolated);

    // Add to logger
    logger_->add("unfiltered_com_pos_ref", Eigen::Vector3d(mpc_res_.com(1).x(), mpc_res_.com(1).y(), mpc_res_.com(1).z()));
    logger_->add("unfiltered_com_vel_ref", Eigen::Vector3d(mpc_res_.com_vel(1).xdot(), mpc_res_.com_vel(1).ydot(), mpc_res_.com_vel(1).zdot()));
    logger_->add("unfiltered_com_acc_ref", Eigen::Vector3d(mpc_res_.com_acc(1).xddot(), mpc_res_.com_acc(1).yddot(), mpc_res_.com_acc(1).zddot()));
    logger_->add("com_pos_ref", tci_container_->task_map_["com_task"]->DesiredPos());
    logger_->add("com_vel_ref", tci_container_->task_map_["com_task"]->DesiredVel());
    logger_->add("com_acc_ref", tci_container_->task_map_["com_task"]->DesiredAcc());
    logger_->add("actual_com_pos", robot_->GetRobotComPos());
    logger_->add("actual_com_vel", robot_->GetRobotComLinVel());
    logger_->add("torso_ori_ref", Eigen::Vector4d(mpc_res_.ori(1).x(), mpc_res_.ori(1).y(), mpc_res_.ori(1).z(), mpc_res_.ori(1).w()));
    logger_->add("torso_vel", Eigen::Vector3d(mpc_res_.omega(1).x(), mpc_res_.omega(1).y(), mpc_res_.omega(1).z()));
    logger_->add("unfiltered_lf_pos_ref", std::get<0>(left_foot_ref));
    logger_->add("unfiltered_lf_vel_ref", std::get<1>(left_foot_ref));
    logger_->add("unfiltered_lf_acc_ref", std::get<2>(left_foot_ref));
    logger_->add("lf_pos_ref", lf_pos_ref);
    logger_->add("lf_vel_ref", lf_vel_ref);
    logger_->add("lf_acc_ref", lf_acc_ref);
    logger_->add("actual_lf_pos", robot_->GetLinkIsometry(draco_link::l_foot_contact).translation());
    logger_->add("unfiltered_left_force_ref", left_force_ref);
    logger_->add("left_force_ref", _LinearInterpolation(old_lf_force_, left_force_ref));
    logger_->add("unfiltered_rf_pos_ref", std::get<0>(right_foot_ref));
    logger_->add("unfiltered_rf_vel_ref", std::get<1>(right_foot_ref));
    logger_->add("unfiltered_rf_acc_ref", std::get<2>(right_foot_ref));
    logger_->add("rf_pos_ref", rf_pos_ref);
    logger_->add("rf_vel_ref", rf_vel_ref);
    logger_->add("rf_acc_ref", rf_acc_ref);
    logger_->add("actual_rf_pos", robot_->GetLinkIsometry(draco_link::r_foot_contact).translation());
    logger_->add("unfiltered_right_force_ref", right_force_ref);
    logger_->add("right_force_ref", _LinearInterpolation(old_rf_force_, right_force_ref));
    logger_->add("time", count_ * ctrl_dt_);
    logger_->add("count", count_);

    interp_count_++;

    return true;
}


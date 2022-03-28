#pragma once

#include <iostream>
#include <string>
#include <Eigen/Dense>
#include <vector>

#include "pnc/whole_body_controllers/task.hpp"
#include "pnc/robot_system/robot_system.hpp"

class JointTask : public Task {
    public:
        JointTask(RobotSystem *_robot);
        virtual ~JointTask();

        void UpdateOscCommand();

        void UpdateTaskJacobian();
        void UpdateTaskJacobianDotQdot();
};

class SelectedJointTask : public Task{
    public:
        SelectedJointTask(RobotSystem *_robot, 
                const std::vector<std::string> &_joint_container);
        virtual ~SelectedJointTask();

        void UpdateOscCommand();

        void UpdateTaskJacobian();
        void UpdateTaskJacobianDotQdot();
    private:
        std::vector<std::string> joint_container_;
};

class LinkPosTask : public Task{
    public:
        LinkPosTask(RobotSystem *_robot, const std::string &_target_link);
        virtual ~LinkPosTask();

        void UpdateOscCommand();

        void UpdateTaskJacobian();
        void UpdateTaskJacobianDotQdot();
    private:
        std::string target_link_;

};

class LinkOriTask : public Task{
    public:
        LinkOriTask(RobotSystem *_robot, const std::string &_target_link);
        virtual ~LinkOriTask();

        void UpdateOscCommand();

        void UpdateTaskJacobian();
        void UpdateTaskJacobianDotQdot();
    private:
        std::string target_link_;
};

class ComTask : public Task{
    public:
        ComTask(RobotSystem *_robot);
        virtual ~ComTask();

        void UpdateOscCommand();

        void UpdateTaskJacobian();
        void UpdateTaskJacobianDotQdot();
};

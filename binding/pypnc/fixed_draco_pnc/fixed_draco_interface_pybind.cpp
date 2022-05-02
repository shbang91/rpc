#include<pybind11/pybind11.h>
#include<pybind11/stl.h>
#include<pybind11/eigen.h>

#include "pnc/fixed_draco_pnc/fixed_draco_interface.hpp"

//overriding virtual function
class PyInterface : public Interface {
    public:
        using Interface::Interface;
        void GetCommand(void *_sensor_data, void *_command_data) override {
            PYBIND11_OVERRIDE_PURE(
                    void,
                    Interface,
                    GetCommand,
                    _sensor_data,
                    _command_data
                    );
        }
};

namespace py = pybind11;

PYBIND11_MODULE(fixed_draco_interface_pybind, m){
    py::class_<Interface, PyInterface>(m, "Interface")
        .def(py::init<>())
        .def("GetCommand", &Interface::GetCommand);

    py::class_<FixedDracoInterface, Interface>(m, "FixedDracoInterface")
        .def(py::init<>());

    py::class_<FixedDracoSensorData>(m, "FixedDracoSensorData")
        .def(py::init<>())
        .def_readwrite("joint_positions_", &FixedDracoSensorData::joint_positions_)
        .def_readwrite("joint_velocities_", &FixedDracoSensorData::joint_velocities_)
        .def_readwrite("imu_frame_isometry_", &FixedDracoSensorData::imu_frame_isometry_)
        .def_readwrite("imu_frame_velocities_", &FixedDracoSensorData::imu_frame_velocities_)

        .def_readwrite("base_com_pos_", &FixedDracoSensorData::base_com_pos_)
        .def_readwrite("base_com_quat_", &FixedDracoSensorData::base_com_quat_)
        .def_readwrite("base_com_lin_vel_", &FixedDracoSensorData::base_com_lin_vel_)
        .def_readwrite("base_com_ang_vel_", &FixedDracoSensorData::base_com_ang_vel_)

        .def_readwrite("base_joint_pos_", &FixedDracoSensorData::base_joint_pos_)
        .def_readwrite("base_joint_quat_", &FixedDracoSensorData::base_joint_quat_)
        .def_readwrite("base_joint_lin_vel_", &FixedDracoSensorData::base_joint_lin_vel_)
        .def_readwrite("base_joint_ang_vel_", &FixedDracoSensorData::base_joint_ang_vel_);

    py::class_<FixedDracoCommand>(m, "FixedDracoCommand")
        .def(py::init<>())
        .def_readwrite("joint_positions_cmd_", &FixedDracoCommand::joint_positions_cmd_)
        .def_readwrite("joint_velocities_cmd_", &FixedDracoCommand::joint_velocities_cmd_)
        .def_readwrite("joint_torques_cmd_", &FixedDracoCommand::joint_torques_cmd_);


}

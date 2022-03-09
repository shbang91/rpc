#include<pybind11/pybind11.h>
#include<pybind11/stl.h>
#include<pybind11/eigen.h>

#include "pnc/draco_pnc/draco_interface.hpp"

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

PYBIND11_MODULE(draco_interface_pybind, m){
    py::class_<Interface, PyInterface>(m, "Interface")
        .def(py::init<>())
        .def("GetCommand", &Interface::GetCommand);

    py::class_<DracoInterface, Interface>(m, "DracoInterface")
        .def(py::init<>());

    py::class_<DracoSensorData>(m, "DracoSensorData")
        .def(py::init<>())
        .def_readwrite("joint_positions_", &DracoSensorData::joint_positions_)
        .def_readwrite("joint_velocities_", &DracoSensorData::joint_velocities_)
        .def_readwrite("imu_frame_isometry_", &DracoSensorData::imu_frame_isometry_)
        .def_readwrite("imu_frame_velocities_", &DracoSensorData::imu_frame_velocities_)
        .def_readwrite("b_lf_contact_", &DracoSensorData::b_lf_contact_)
        .def_readwrite("b_rf_contact_", &DracoSensorData::b_rf_contact_);

    py::class_<DracoCommand>(m, "DracoCommand")
        .def(py::init<>())
        .def_readwrite("joint_positions_cmd_", &DracoCommand::joint_positions_cmd_)
        .def_readwrite("joint_velocities_cmd_", &DracoCommand::joint_velocities_cmd_)
        .def_readwrite("joint_torques_cmd_", &DracoCommand::joint_torques_cmd_);


}

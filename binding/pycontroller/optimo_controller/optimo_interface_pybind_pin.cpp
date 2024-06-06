#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "controller/optimo_controller/optimo_interface.hpp"
#include "controller/interrupt_handler.hpp"

class PyInterface : public Interface {
public:
  using Interface::Interface;
  void GetCommand(void *sensor_data, void *command_data) override {
    PYBIND11_OVERRIDE_PURE(void, Interface, GetCommand, sensor_data,
                           command_data);
  }
};

namespace py = pybind11;
PYBIND11_MODULE(optimo_interface_py, m){
    py::module::import("interrupt_py");

    py::class_<Interface, PyInterface>(m, "Interface")
        .def(py::init<>())
        .def("GetCommand", &Interface::GetCommand);

    py::class_<OptimoInterface, Interface>(m, "OptimoInterface")
        .def(py::init<>())
        .def_readwrite("interrupt_", &OptimoInterface::interrupt_handler_);
    
    py::class_<OptimoSensorData>(m, "OptimoSensorData")
        .def(py::init<>())
        .def_readwrite("joint_pos_", &OptimoSensorData::joint_pos_)
        .def_readwrite("joint_vel_", &OptimoSensorData::joint_vel_)
        .def_readwrite("joint_trq_", &OptimoSensorData::joint_trq_)
        .def_readwrite("joint_sea_trq_", &OptimoSensorData::joint_sea_trq_)
        .def_readwrite("base_pos_", &OptimoSensorData::base_pos_)
        .def_readwrite("base_quat_", &OptimoSensorData::base_quat_);

    py::class_<OptimoCommand>(m, "OptimoCommand")
        .def(py::init<>())
        .def_readwrite("joint_pos_cmd_", &OptimoCommand::joint_pos_cmd_)
        .def_readwrite("joint_vel_cmd_", &OptimoCommand::joint_vel_cmd_)
        .def_readwrite("joint_trq_cmd_", &OptimoCommand::joint_trq_cmd_);
        
}
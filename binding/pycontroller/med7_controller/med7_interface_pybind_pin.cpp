#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "controller/med7_controller/med7_interface.hpp"
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
PYBIND11_MODULE(med7_interface_py, m){
    py::module::import("interrupt_py");

    py::class_<Interface, PyInterface>(m, "Interface")
        .def(py::init<>())
        .def("GetCommand", &Interface::GetCommand);

    py::class_<Med7Interface, Interface>(m, "Med7Interface")
        .def(py::init<>())
        .def_readwrite("interrupt_", &Med7Interface::interrupt_handler_);
    
    py::class_<Med7SensorData>(m, "Med7SensorData")
        .def(py::init<>())
        .def_readwrite("joint_pos_", &Med7SensorData::joint_pos_)
        .def_readwrite("joint_vel_", &Med7SensorData::joint_vel_)
        .def_readwrite("joint_trq_", &Med7SensorData::joint_trq_)
        .def_readwrite("joint_sea_trq_", &Med7SensorData::joint_sea_trq_)
        .def_readwrite("base_pos_", &Med7SensorData::base_pos_)
        .def_readwrite("base_quat_", &Med7SensorData::base_quat_);

    py::class_<Med7Command>(m, "Med7Command")
        .def(py::init<>())
        .def_readwrite("joint_pos_cmd_", &Med7Command::joint_pos_cmd_)
        .def_readwrite("joint_vel_cmd_", &Med7Command::joint_vel_cmd_)
        .def_readwrite("joint_trq_cmd_", &Med7Command::joint_trq_cmd_);
        
}

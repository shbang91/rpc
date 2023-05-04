#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "controller/draco_controller/draco_interface.hpp"
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

PYBIND11_MODULE(draco_interface_py, m) {
  py::module::import("interrupt_py");

  py::class_<Interface, PyInterface>(m, "Interface")
      .def(py::init<>())
      .def("GetCommand", &Interface::GetCommand);

  py::class_<DracoInterface, Interface>(m, "DracoInterface")
      .def(py::init<>())
      .def_readwrite("interrupt_", &DracoInterface::interrupt_handler_);

  py::class_<DracoSensorData>(m, "DracoSensorData")
      .def(py::init<>())
      .def_readwrite("imu_frame_quat_", &DracoSensorData::imu_frame_quat_)
      .def_readwrite("imu_ang_vel_", &DracoSensorData::imu_ang_vel_)
      .def_readwrite("joint_pos_", &DracoSensorData::joint_pos_)
      .def_readwrite("joint_vel_", &DracoSensorData::joint_vel_)
      .def_readwrite("b_lf_contact_", &DracoSensorData::b_lf_contact_)
      .def_readwrite("b_rf_contact_", &DracoSensorData::b_rf_contact_)
      .def_readwrite("lf_contact_normal_", &DracoSensorData::lf_contact_normal_)
      .def_readwrite("rf_contact_normal_", &DracoSensorData::rf_contact_normal_)
      .def_readwrite("imu_dvel_", &DracoSensorData::imu_dvel_)

      // Debug
      .def_readwrite("base_joint_pos_", &DracoSensorData::base_joint_pos_)
      .def_readwrite("base_joint_quat_", &DracoSensorData::base_joint_quat_)
      .def_readwrite("base_joint_lin_vel_",
                     &DracoSensorData::base_joint_lin_vel_)
      .def_readwrite("base_joint_ang_vel_",
                     &DracoSensorData::base_joint_ang_vel_);

  py::class_<DracoCommand>(m, "DracoCommand")
      .def(py::init<>())
      .def_readwrite("joint_pos_cmd_", &DracoCommand::joint_pos_cmd_)
      .def_readwrite("joint_vel_cmd_", &DracoCommand::joint_vel_cmd_)
      .def_readwrite("joint_trq_cmd_", &DracoCommand::joint_trq_cmd_);
}

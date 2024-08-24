#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include "controller/go2_controller/go2_interface.hpp"
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

PYBIND11_MODULE(go2_interface_py, m) {
  // py::module::import("interrupt_py");

  py::class_<Interface, PyInterface>(m, "Interface")
      .def(py::init<>())
      .def("GetCommand", &Interface::GetCommand);

  py::class_<Go2Interface, Interface>(m, "Go2Interface").def(py::init<>());
  //.def_readwrite("interrupt_", &Go2Interface::interrupt_handler_);

  py::class_<Go2SensorData>(m, "Go2SensorData")
      .def(py::init<>())
      .def_readwrite("imu_frame_quat_", &Go2SensorData::imu_frame_quat_)
      .def_readwrite("imu_ang_vel_", &Go2SensorData::imu_ang_vel_)
      .def_readwrite("joint_pos_", &Go2SensorData::joint_pos_)
      .def_readwrite("joint_vel_", &Go2SensorData::joint_vel_)
      .def_readwrite("b_FL_foot_contact_", &Go2SensorData::b_FL_foot_contact_)
      .def_readwrite("b_FR_foot_contact_", &Go2SensorData::b_FR_foot_contact_)
      .def_readwrite("b_RL_foot_contact_", &Go2SensorData::b_RL_foot_contact_)
      .def_readwrite("b_RR_foot_contact_", &Go2SensorData::b_RR_foot_contact_)
      .def_readwrite("FL_normal_force_", &Go2SensorData::FL_normal_force_)
      .def_readwrite("FR_normal_force_", &Go2SensorData::FR_normal_force_)
      .def_readwrite("RL_normal_force_", &Go2SensorData::RL_normal_force_)
      .def_readwrite("RR_normal_force_", &Go2SensorData::RR_normal_force_)
      .def_readwrite("imu_dvel_", &Go2SensorData::imu_dvel_)
      .def_readwrite("imu_lin_acc_", &Go2SensorData::imu_lin_acc_)

      // Debug
      .def_readwrite("base_joint_pos_", &Go2SensorData::base_joint_pos_)
      .def_readwrite("base_joint_quat_", &Go2SensorData::base_joint_quat_)
      .def_readwrite("base_joint_lin_vel_", &Go2SensorData::base_joint_lin_vel_)
      .def_readwrite("base_joint_ang_vel_",
                     &Go2SensorData::base_joint_ang_vel_);

  py::class_<Go2Command>(m, "Go2Command")
      .def(py::init<>())
      .def_readwrite("joint_pos_cmd_", &Go2Command::joint_pos_cmd_)
      .def_readwrite("joint_vel_cmd_", &Go2Command::joint_vel_cmd_)
      .def_readwrite("joint_trq_cmd_", &Go2Command::joint_trq_cmd_);
}

#include <pybind11/pybind11.h>

#include "controller/interrupt_handler.hpp"

namespace py = pybind11;

PYBIND11_MODULE(interrupt_py, m) {
  py::class_<InterruptHandler>(m, "InterruptHandler")
      .def(py::init<>())
      .def("PressOne", &InterruptHandler::PressOne)
      .def("PressTwo", &InterruptHandler::PressTwo)
      .def("PressFour", &InterruptHandler::PressFour)
      .def("PressFive", &InterruptHandler::PressFive)
      .def("PressSix", &InterruptHandler::PressSix)
      .def("PressSeven", &InterruptHandler::PressSeven)
      .def("PressEight", &InterruptHandler::PressEight)
      .def("PressNine", &InterruptHandler::PressNine)
      .def("Pressa", &InterruptHandler::Pressa);
}

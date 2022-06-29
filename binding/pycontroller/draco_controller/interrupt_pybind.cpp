#include <pybind11/pybind11.h>

#include "controller/interrupt.hpp"

namespace py = pybind11;

PYBIND11_MODULE(interrupt_py, m) {
  py::class_<Interrupt>(m, "Interrupt")
      .def(py::init<>())
      .def("PressOne", &Interrupt::PressOne);
}

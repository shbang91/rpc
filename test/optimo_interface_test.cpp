#include <gtest/gtest.h>

#include "controller/optimo_controller/optimo_interface.hpp"
#include <iostream>

int main(int argc, char **argv) {

  Interface *optimo_interface = new OptimoInterface();
  delete optimo_interface;

  return 0;
}


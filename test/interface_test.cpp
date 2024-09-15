#include "pnc/draco_pnc/draco_interface.hpp"

// #include <cstdlib>
// #include <iostream>
// #include <unistd.h>

int main() {

  // Interface *interface_ = new DracoInterface();
  // interface_->print();
  // interface_->print2(); //error might occur

  DracoInterface *interface2_ = new DracoInterface();
  std::cout << "construct DracoInterface" << std::endl;
  delete interface2_;
  std::cout << "destruct DracoInterface" << std::endl;
  // interface2_->print();
  // interface2_->print2();

  return 0;
}

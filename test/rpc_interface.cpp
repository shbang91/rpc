#include "controller/draco_controller/draco_interface.hpp"

int main() {
  for (int i = 0; i < 100; i++) {
    std::cout << "count: " << i << std::endl;
    DracoInterface *interface = new DracoInterface();
    std::cout << "DracoInterface Constructed" << std::endl;

    // DracoSensorData *sensor_data = new DracoSensorData();
    // DracoCommand *command = new DracoCommand();

    delete interface;
    // delete sensor_data;
    // delete command;
    std::cout << "DracoInterface Destructed" << std::endl;
    std::cout << "--------------------------------" << std::endl;
  }

  return 0;
}

file(GLOB pin_robot_system_source "pinocchio_robot_system.cpp")
file(GLOB pin_robot_system_header "pinocchio_robot_system.hpp")

add_library(pin-robot-system SHARED ${pin_robot_system_source}
    ${pin_robot_system_header})
target_link_libraries(pin-robot-system ${PINOCCHIO_LIBRARIES}
                                            rpc-util)

install(TARGETS pin-robot-system DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${pin_robot_system_header} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/robot_system")

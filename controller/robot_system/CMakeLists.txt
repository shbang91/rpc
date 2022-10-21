file(GLOB pin_robot_system_source "pinocchio_robot_system.cpp")
file(GLOB pin_robot_system_header "pinocchio_robot_system.hpp")
file(GLOB dart_robot_system_source "dart_robot_system.cpp")
file(GLOB dart_robot_system_header "dart_robot_system.hpp")

add_library(rpc-pin-robot-system STATIC ${pin_robot_system_source}
    ${pin_robot_system_header})
#target_link_libraries(rpc-pin-robot-system ${pinocchio_LIBRARIES}
                                            #rpc-util)
target_link_libraries(rpc-pin-robot-system PUBLIC pinocchio::pinocchio
                                           PUBLIC rpc-util)

#add_library(rpc-dart-robot-system ${dart_robot_system_source}
    #${dart_robot_system_header})
#target_link_libraries(rpc-dart-robot-system ${DART_LIBRARIES}
                                            #rpc-util)

install(TARGETS rpc-pin-robot-system DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${pin_robot_system_header} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/robot_system")

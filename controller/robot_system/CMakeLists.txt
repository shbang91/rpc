file(GLOB pin_robot_system_source "pinocchio_robot_system.cpp")
file(GLOB pin_robot_system_header "pinocchio_robot_system.hpp")

if(BUILD_WITH_DART)
    file(GLOB dart_robot_system_source "dart_robot_system.cpp")
    file(GLOB dart_robot_system_header "dart_robot_system.hpp")
endif()

add_library(rpc-pin-robot-system ${pin_robot_system_source}
    ${pin_robot_system_header})
#target_link_libraries(rpc-pin-robot-system ${pinocchio_LIBRARIES}
                                            #rpc-util)
target_link_libraries(rpc-pin-robot-system PUBLIC pinocchio::pinocchio
                                           PUBLIC rpc-util)

if(BUILD_WITH_DART)
    add_library(rpc-dart-robot-system ${dart_robot_system_source} ${dart_robot_system_header})
    target_link_libraries(rpc-dart-robot-system PUBLIC ${DART_LIBRARIES}
                                                PUBLIC rpc-util)
endif()

install(TARGETS rpc-pin-robot-system DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${pin_robot_system_header} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/robot_system")

if(BUILD_WITH_DART)
    install(TARGETS rpc-dart-robot-system DESTINATION "${INSTALL_LIB_DIR}")
    install(FILES ${dart_robot_system_header} DESTINATION "${INSTALL_INCLUDE_DIR}/controller/robot_system")
endif()

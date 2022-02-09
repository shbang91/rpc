file(GLOB robot_system_sources "*.cpp")
file(GLOB robot_system_headers "*.hpp")

add_library(robot-system ${robot_system_sources} ${robot_system_headers})

target_link_libraries(robot-system ${DART_LIBRARIES})

install(TARGETS robot-system DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${robot_system_headers} DESTINATION
    "{INSTALL_INCLUDE_DIR}/pnc/robot_system")

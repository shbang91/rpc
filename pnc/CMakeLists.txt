add_subdirectory(draco_pnc)
add_subdirectory(robot_system)
add_subdirectory(whole_body_controllers)

file(GLOB pnc_headers "*.hpp")
install(FILES pnc_headers DESTINATION "${INSTALL_INCLUDE_DIR}/pnc")

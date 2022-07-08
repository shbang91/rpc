add_subdirectory(draco_controller)
add_subdirectory(robot_system)
add_subdirectory(whole_body_controller)
add_subdirectory(model_predictive_controller)

file(GLOB rpc_headers "*.hpp")
install(FILES ${rpc_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/controller")

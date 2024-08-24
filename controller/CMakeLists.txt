add_subdirectory(draco_controller)
add_subdirectory(go2_controller)
add_subdirectory(robot_system)
add_subdirectory(whole_body_controller)
add_subdirectory(filter)
add_subdirectory(state_estimator)
add_subdirectory(friction_compensator)
add_subdirectory(models)


file(GLOB rpc_headers "*.hpp")
install(FILES ${rpc_headers} DESTINATION "${INSTALL_INCLUDE_DIR}/controller")

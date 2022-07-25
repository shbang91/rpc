file(GLOB_RECURSE sources "*.cpp")
file(GLOB controller_headers "*.hpp")
file(GLOB state_machines_headers "draco_state_machines/*.hpp")
file(GLOB draco_task_headers "draco_task/*.hpp")

add_library(draco-controller SHARED ${sources} ${controller_headers}
    ${state_machines_headers} ${draco_task_headers})
target_link_libraries(draco-controller pin-robot-system
                                       ${Protobuf_LIBRARIES}
                                       ${ZMQ_LIBRARIES}
                                       draco-rpc-msg
                                       #pnc-to-horizon
                                       horizon-to-pnc
                                       nmpc-handler
                                       rpc-wbc)

install(TARGETS draco-controller DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${controller_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/draco_controller")
install(FILES ${state_machines_headers} DESTINATION
    "{INSTALL_INCLUDE_DIR}/controller/draco_controller/draco_state_machines")
install(FILES ${draco_task_headers} DESTINATION
    "{INSTALL_INCLUDE_DIR}/controller/draco_controller/draco_task")


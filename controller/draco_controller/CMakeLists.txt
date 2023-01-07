file(GLOB_RECURSE sources "*.cpp")
file(GLOB controller_headers "*.hpp")
file(GLOB state_machines_headers "draco_state_machines/*.hpp")
file(GLOB draco_task_headers "draco_task/*.hpp")

add_library(rpc-draco-controller SHARED ${sources} ${controller_headers}
    ${state_machines_headers} ${draco_task_headers})
#target_link_libraries(rpc-draco-controller ${PINOCCHIO_LIBRARIES}
                                        #${Protobuf_LIBRARIES}
                                        #${ZMQ_LIBRARIES}
                                        #rpc-draco-msg
                                        #rpc-pin-robot-system
                                        #rpc-wbc
                                        #rpc-util
                                        #rpc-dcm-planner
                                        #rpc-lmpc-handler
                                        #rpc-filter)
target_link_libraries(rpc-draco-controller PUBLIC ${Protobuf_LIBRARIES}
                                           PUBLIC ${ZMQ_LIBRARIES}
                                           PUBLIC rpc-draco-msg
                                           PUBLIC rpc-pin-robot-system
                                           PUBLIC rpc-wbc
                                           PUBLIC rpc-util
                                           PUBLIC rpc-dcm-planner
                                           PUBLIC rpc-lmpc-handler
                                           PUBLIC rpc-filter)
if(BUILD_WITH_MATLOGGER)
    target_link_libraries(rpc-draco-controller PUBLIC matlogger2::matlogger2)
endif()


install(TARGETS rpc-draco-controller DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${controller_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/draco_controller")
install(FILES ${state_machines_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/draco_controller/draco_state_machines")
install(FILES ${draco_task_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/draco_controller/draco_task")



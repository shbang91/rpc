file(GLOB_RECURSE sources "*.cpp")
file(GLOB controller_headers "*.hpp")

add_library(draco-controller SHARED ${sources})
target_link_libraries(draco-controller pin-robot-system
                                        ${Protobuf_LIBRARIES}
                                        ${ZMQ_LIBRARIES}
                                        draco-rpc-msg)

install(TARGETS draco-controller DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${controller_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/draco_controller")



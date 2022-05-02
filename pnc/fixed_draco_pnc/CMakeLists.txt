file(GLOB_RECURSE sources "*.cpp")
file(GLOB fixed_draco_pnc_headers "*.hpp")

add_library(fixed-draco-pnc SHARED ${sources})
target_link_libraries(fixed-draco-pnc ${DART_LIBRARIES}
                                ${Protobuf_LIBRARIES}
                                ${ZMQ_LIBRARIES}
                                robot-system
                                #fixed-draco-pnc-msg
                                pnc-util)

install(TARGETS fixed-draco-pnc DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${fixed_draco_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/fixed_draco_pnc")


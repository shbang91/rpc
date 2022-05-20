file(GLOB_RECURSE sources "*.cpp")
file(GLOB draco_pnc_headers "*.hpp")

add_library(draco-pnc SHARED ${sources})
target_link_libraries(draco-pnc ${Protobuf_LIBRARIES}
                                ${ZMQ_LIBRARIES}
                                dart-robot-system
                                pinocchio-robot-system
                                draco-pnc-msg
                                pnc-util)

install(TARGETS draco-pnc DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${draco_pnc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/pnc/draco_pnc")


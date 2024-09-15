file(GLOB_RECURSE sources "*.cpp" "*.c")
file(GLOB controller_headers "*.hpp")

add_library(rpc-go2-controller SHARED ${sources} ${controller_headers})
target_link_libraries(rpc-go2-controller PUBLIC rpc-pin-robot-system
                                           PUBLIC rpc-wbc
                                           PUBLIC rpc-util
                                           )


install(TARGETS rpc-go2-controller DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${controller_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/go2_controller")

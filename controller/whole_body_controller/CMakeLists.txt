FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB wbc_headers *.hpp)

add_library(rpc-wbc ${sources})

target_link_libraries(rpc-wbc pin-robot-system
                              pnc-util)

install(TARGETS rpc-wbc DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${wbc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/whole_body_controllers")


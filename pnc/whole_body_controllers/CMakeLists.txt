FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB wbc_headers *.hpp)

add_library(pnc-wbc ${sources})

target_link_libraries(pnc-wbc robot-system
                              pnc-util)

install(TARGETS pnc-wbc DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${wbc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/whole_body_controllers")


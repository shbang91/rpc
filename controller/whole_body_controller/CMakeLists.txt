FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB wbc_headers *.hpp)
FILE(GLOB manager_headers managers/*.hpp)

add_library(rpc-wbc SHARED ${sources} ${wbc_headers} ${manager_headers})

target_link_libraries(rpc-wbc pin-robot-system
                              goldfarb
                              rpc-util
                              dcm_planner)

install(TARGETS rpc-wbc DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${wbc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/whole_body_controller")
install(FILES ${wbc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/whole_body_controller/managers")


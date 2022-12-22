FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB wbc_headers *.hpp)
FILE(GLOB manager_headers managers/*.hpp)

add_library(rpc-wbc ${sources} ${wbc_headers} ${manager_headers})

target_link_libraries(rpc-wbc PUBLIC rpc-pin-robot-system
                              PUBLIC rpc-goldfarb
                              PUBLIC rpc-dcm-planner
                              PUBLIC rpc-util)
if(BUILD_WITH_MATLOGGER)
    target_link_libraries(rpc-wbc PUBLIC matlogger2::matlogger2)
endif()

install(TARGETS rpc-wbc DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${wbc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/whole_body_controller")
install(FILES ${manager_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/whole_body_controller/managers")


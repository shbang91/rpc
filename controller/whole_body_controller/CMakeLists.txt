FILE(GLOB_RECURSE sources *.cpp)
FILE(GLOB wbc_headers *.hpp)
FILE(GLOB ihwbc_headers ihwbc/*.hpp)
FILE(GLOB wbic_headers wbic/*.hpp)
FILE(GLOB manager_headers managers/*.hpp)

add_library(rpc-wbc ${sources} ${wbc_headers} ${ihwbc_headers} ${wbic_headers} ${manager_headers})

target_link_libraries(rpc-wbc PUBLIC rpc-pin-robot-system
                              PUBLIC rpc-goldfarb
                              PUBLIC rpc-dcm-planner
                              PUBLIC rpc-util)
if(BUILD_WITH_MATLOGGER)
    target_link_libraries(rpc-wbc PUBLIC matlogger2::matlogger2)
endif()

if(BUILD_WITH_PROXQP)
    #target_link_libraries(rpc-wbc PUBLIC proxsuite::proxsuite)
    target_link_libraries(rpc-wbc PUBLIC proxsuite::proxsuite-vectorized)
    #target_compile_options(rpc-wbc PUBLIC "-march=native")
endif()

install(TARGETS rpc-wbc DESTINATION "${INSTALL_LIB_DIR}")
install(FILES ${wbc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/whole_body_controller")
install(FILES ${ihwbc_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/whole_body_controller/ihwbc")
install(FILES ${wbic_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/whole_body_controller/wbic")
install(FILES ${manager_headers} DESTINATION
    "${INSTALL_INCLUDE_DIR}/controller/whole_body_controller/managers")
